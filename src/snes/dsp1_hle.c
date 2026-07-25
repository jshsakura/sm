/* DSP-1 HLE — see dsp1_hle.h for provenance. Command set and word counts from
 * public documentation (nesdev/sneslab); arithmetic implemented fresh from the
 * operations' definitions (Q15 fixed point, angles with 0x10000 = full turn).
 *
 * Precision stance: the real chip computes in 16-bit fixed point with its own
 * rounding. This HLE computes in double and rounds once, which is at least as
 * accurate; games tolerate this (they feed the results to the PPU or compare
 * magnitudes). If a title ever proves sensitive, tune per command.
 *
 * The projection group (Parameter/Raster/Project/Target) implements a single
 * coherent pinhole-over-plane model; the constants are geometric, not copied.
 * Good enough to drive Mode-7 sensibly; refine against hardware captures if a
 * game's horizon sits visibly wrong. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dsp1_hle.h"

#define DSP1_SAVE_VERSION 1

static const double TAU = 6.283185307179586;

static double angle(uint16_t a) { return (double)a * (TAU / 65536.0); }

static int16_t clamp16(double v) {
  if (v > 32767.0) return 32767;
  if (v < -32768.0) return -32768;
  return (int16_t)lround(v);
}

void dsp1_reset(Dsp1* d) {
  memset(d, 0, sizeof(*d));
  d->version = DSP1_SAVE_VERSION;
  d->lfe = 0x0100;               /* sane non-zero defaults until Parameter runs */
  d->fz = 0x0100;
}

uint8_t dsp1_readSR(Dsp1* d) {
  /* bit7 RQM: always ready (commands execute instantly).
   * bit4 DRS: a 16-bit transfer's second half is pending — games use this to
   * resync the byte stream after interrupts (Mario Kart checks it). */
  return 0x80 | ((d->byteIdx & 1) ? 0x10 : 0);
}

/* ---- command implementations ------------------------------------------- */

static void attitude(Dsp1* d, int slot) {
  /* in: m, Az, Ay, Ax -> build scaled rotation matrix (coordinate transform) */
  double m = (double)d->in[0] / 32768.0;
  double az = angle((uint16_t)d->in[1]);
  double ay = angle((uint16_t)d->in[2]);
  double ax = angle((uint16_t)d->in[3]);
  double cz = cos(az), sz = sin(az);
  double cy = cos(ay), sy = sin(ay);
  double cx = cos(ax), sx = sin(ax);
  /* R = Rx * Ry * Rz applied to row vectors: world -> object axes */
  double r[3][3] = {
    { cy * cz,                cy * sz,               -sy      },
    { sx * sy * cz - cx * sz, sx * sy * sz + cx * cz, sx * cy },
    { cx * sy * cz + sx * sz, cx * sy * sz - sx * cz, cx * cy },
  };
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      d->matrix[slot][i][j] = clamp16(r[i][j] * m * 32768.0);
}

static void objective(Dsp1* d, int slot) {
  /* world (X,Y,Z) -> object frame (F,L,U): rows of M dot v */
  for (int i = 0; i < 3; i++) {
    double acc = 0;
    for (int j = 0; j < 3; j++)
      acc += (double)d->matrix[slot][i][j] * d->in[j];
    d->out[i] = clamp16(acc / 32768.0);
  }
}

static void subjective(Dsp1* d, int slot) {
  /* object frame -> world: transpose (rotation matrices: inverse = transpose) */
  for (int i = 0; i < 3; i++) {
    double acc = 0;
    for (int j = 0; j < 3; j++)
      acc += (double)d->matrix[slot][j][i] * d->in[j];
    d->out[i] = clamp16(acc / 32768.0);
  }
}

static void scalar(Dsp1* d, int slot) {
  /* forward-axis component of (X,Y,Z) in the object frame */
  double acc = 0;
  for (int j = 0; j < 3; j++)
    acc += (double)d->matrix[slot][0][j] * d->in[j];
  d->out[0] = clamp16(acc / 32768.0);
}

/* Ground-plane camera model shared by Raster / Project / Target.
 * Camera sits at (fx, fy) height fz, pitched down by `aas`, yawed by `azs`;
 * screen plane at distance lfe. Screen line v (0-based raster) looks along a
 * ray pitched by pitch + atan((v - vof)/lfe); where that ray meets the ground
 * is at horizontal distance dist = fz / tan(ray). */

/* Polynomial atan2 that avoids libm's atan/atan2 entirely.
 *
 * WHY: libm's atan is captured by the NES overlay linker rule
 * (.overlay_nes_fceu: *libm.a:libm_a-s_atan.o), so calling it from the SNES
 * overlay resolves to the NES overlay's VMA — stale when SNES is loaded →
 * Busfault. atan2 is not captured but its wrapper objects (e_atan2.o,
 * w_atan2.o) overflow internal flash. This polynomial lives entirely in
 * the SNES overlay and has max error ~5e-5 rad — invisible at the DSP-1's
 * 16-bit fixed-point precision (1 LSB ≈ 2.7e-4 rad).
 *
 * Coefficients from Carlson's minimax fit (Abramowitz & Stegun §4.4.49). */
static double dsp_atan2(double y, double x) {
  /* x is always positive (lfe clamped to >= 1 at every call site), so this
   * reduces to atan(y/x) with quadrant 0 or ±π. */
  double t = y / x;
  double a;
  if (t > 1.0)       { t = 1.0 / t; a = 1.5707963 - t * (0.9998660 + t*t*(-0.3302995 + t*t*(0.1801410 + t*t*(-0.0851330 + t*t*0.0208351)))); }
  else if (t < -1.0) { t = 1.0 / t; a = -1.5707963 - t * (0.9998660 + t*t*(-0.3302995 + t*t*(0.1801410 + t*t*(-0.0851330 + t*t*0.0208351)))); }
  else               { double t2 = t*t; a = t * (0.9998660 + t2*(-0.3302995 + t2*(0.1801410 + t2*(-0.0851330 + t2*0.0208351)))); }
  return a;
}

static double ground_dist(Dsp1* d, double v) {
  double pitch = angle(d->aas);            /* attack angle: >0 pitches down */
  double ray = pitch + dsp_atan2(v, (double)(d->lfe ? d->lfe : 1));
  /* tan(ray) = sin(ray)/cos(ray): sin/cos are already linked for other DSP-1
   * handlers and live in internal flash (not overlay-captured like atan).
   * This avoids pulling in libm's k_tan.o, saving ~500B of internal flash. */
  double t = sin(ray) / cos(ray);
  if (t < 1e-4) t = 1e-4;                  /* above horizon: clamp far */
  return (double)d->fz / t;
}

static void cmd_parameter(Dsp1* d) {
  d->fx = d->in[0]; d->fy = d->in[1]; d->fz = d->in[2];
  d->lfe = d->in[3]; d->les = d->in[4];
  d->aas = (uint16_t)d->in[5]; d->azs = (uint16_t)d->in[6];

  /* horizon raster: ray pitch crosses 0 at v = -tan(pitch)*lfe
   * (sin/cos, not tan() — see ground_dist comment) */
  double pitch = angle(d->aas);
  double vHorizon = -(sin(pitch) / cos(pitch)) * (double)(d->lfe ? d->lfe : 1);
  d->vof = clamp16(vHorizon);
  d->vva = clamp16(vHorizon);              /* same reference in this model */

  /* ground point on the screen-centre ray, les ahead of the eye */
  double dist = (double)d->les;
  double az = angle(d->azs);
  d->centerX = clamp16((double)d->fx + dist * sin(az));
  d->centerY = clamp16((double)d->fy + dist * cos(az));

  d->out[0] = d->vof; d->out[1] = d->vva;
  d->out[2] = d->centerX; d->out[3] = d->centerY;
}

static void raster_line(Dsp1* d, uint16_t vs) {
  /* Mode-7 matrix for scanline vs: rotate by azimuth, scale by ground distance
   * per screen pixel. Matrix entries are 8.8 fixed point ($211b..$211e). */
  double dist = ground_dist(d, (double)vs);
  double scale = dist / (double)(d->lfe ? d->lfe : 1);
  double az = angle(d->azs);
  double a =  cos(az) * scale, b = sin(az) * scale;
  d->out[0] = clamp16(a * 256.0);          /* A */
  d->out[1] = clamp16(b * 256.0);          /* B */
  d->out[2] = clamp16(-b * 256.0);         /* C */
  d->out[3] = clamp16(a * 256.0);          /* D */
}

static void cmd_project(Dsp1* d) {
  /* world (X,Y,Z) -> screen (H,V) + size M */
  double rx = (double)d->in[0] - d->fx;
  double ry = (double)d->in[1] - d->fy;
  double rz = (double)d->in[2] - d->fz;
  double az = angle(d->azs), pitch = angle(d->aas);
  /* yaw into camera frame: forward = +y' */
  double cx =  rx * cos(az) - ry * sin(az);
  double cy =  rx * sin(az) + ry * cos(az);
  /* pitch around x': depth d, up u */
  double depth = cy * cos(pitch) - rz * sin(pitch);
  double up    = cy * sin(pitch) + rz * cos(pitch);
  if (depth < 1.0) depth = 1.0;
  double lfe = (double)(d->lfe ? d->lfe : 1);
  d->out[0] = clamp16(cx * lfe / depth + 128.0);   /* H */
  d->out[1] = clamp16(up * lfe / depth + 96.0);    /* V */
  d->out[2] = clamp16(lfe * 256.0 / depth);        /* M: enlargement ratio, 8.8 */
}

static void cmd_target(Dsp1* d) {
  /* screen (H,V) -> ground (X,Y): invert the per-line ground model */
  double h = (double)d->in[0] - 128.0;
  double v = (double)d->in[1];
  double dist = ground_dist(d, v);
  double lateral = h * dist / (double)(d->lfe ? d->lfe : 1);
  double az = angle(d->azs);
  d->out[0] = clamp16((double)d->fx + dist * sin(az) + lateral * cos(az));
  d->out[1] = clamp16((double)d->fy + dist * cos(az) - lateral * sin(az));
}

static void cmd_inverse(Dsp1* d) {
  /* floating inverse: value = in[0] * 2^in[1]; out mantissa Q15 in [0x4000,0x7fff] */
  int16_t a = d->in[0];
  int16_t e = d->in[1];
  if (a == 0) { d->out[0] = 0x7fff; d->out[1] = 0x7fff; return; }
  /* base is always exactly 2 with an integer exponent -- scalbn (direct
   * exponent manipulation) is exact and avoids pulling the generic pow()
   * (which pulls exp()/log()/log10()/fmod() with it: several KB nothing else
   * in this command needs). */
  double v = scalbn((double)a / 32768.0, e);
  double inv = 1.0 / v;
  int oe = 0;
  double m = fabs(inv);
  while (m >= 1.0) { m /= 2.0; oe++; }
  while (m < 0.5)  { m *= 2.0; oe--; }
  if (inv < 0) m = -m;
  d->out[0] = clamp16(m * 32768.0);
  d->out[1] = (int16_t)oe;
}

static void execute(Dsp1* d) {
  /* the chip's dispatch table has 0x40 entries: the command index is the low
   * six bits (Mario Kart issues 0x80 for Multiply) */
  uint8_t c = d->cmd & 0x3f;
  int hi = (c >> 4) & 0xf;
  switch (c & 0x0f) {
    case 0x1: if (hi <= 2) { attitude(d, hi); return; } break;
    case 0x3: if (hi <= 2) { subjective(d, hi); return; } break;
    case 0xb: if (hi <= 2) { scalar(d, hi); return; } break;
    case 0xd: if (hi <= 2) { objective(d, hi); return; } break;
    default: break;
  }
  switch (c) {
    case 0x00:  /* multiply: (a*b)>>15 */
      d->out[0] = clamp16((double)d->in[0] * d->in[1] / 32768.0);
      return;
    case 0x04: {  /* triangle: r*sin, r*cos */
      double th = angle((uint16_t)d->in[0]);
      double r = (double)d->in[1];
      d->out[0] = clamp16(r * sin(th));
      d->out[1] = clamp16(r * cos(th));
      return;
    }
    case 0x08: {  /* radius: 32-bit (x^2+y^2+z^2)>>15, LSW first */
      double x = d->in[0], y = d->in[1], z = d->in[2];
      double r = (x * x + y * y + z * z) / 32768.0;
      uint32_t r32 = (r >= 4294967295.0) ? 0xffffffffu : (uint32_t)r;
      d->out[0] = (int16_t)(r32 & 0xffff);
      d->out[1] = (int16_t)(r32 >> 16);
      return;
    }
    case 0x0c: {  /* rotate 2D (coordinate system): in A,X,Y */
      double th = angle((uint16_t)d->in[0]);
      double x = d->in[1], y = d->in[2];
      d->out[0] = clamp16(x * cos(th) + y * sin(th));
      d->out[1] = clamp16(-x * sin(th) + y * cos(th));
      return;
    }
    case 0x10: cmd_inverse(d); return;
    case 0x18: {  /* range: (x^2+y^2+z^2-r^2)>>15 */
      double x = d->in[0], y = d->in[1], z = d->in[2], r = d->in[3];
      d->out[0] = clamp16((x * x + y * y + z * z - r * r) / 32768.0);
      return;
    }
    case 0x1c: {  /* polar: rotate (X,Y,Z) by Az,Ay,Ax (coordinate transform) */
      double az = angle((uint16_t)d->in[0]);
      double ay = angle((uint16_t)d->in[1]);
      double ax = angle((uint16_t)d->in[2]);
      double x = d->in[3], y = d->in[4], z = d->in[5];
      double x1 = x * cos(az) + y * sin(az), y1 = -x * sin(az) + y * cos(az);
      double x2 = x1 * cos(ay) - z * sin(ay), z1 = x1 * sin(ay) + z * cos(ay);
      double y2 = y1 * cos(ax) + z1 * sin(ax), z2 = -y1 * sin(ax) + z1 * cos(ax);
      d->out[0] = clamp16(x2); d->out[1] = clamp16(y2); d->out[2] = clamp16(z2);
      return;
    }
    case 0x28: {  /* distance: sqrt(x^2+y^2+z^2) */
      double x = d->in[0], y = d->in[1], z = d->in[2];
      d->out[0] = clamp16(sqrt(x * x + y * y + z * z));
      return;
    }
    case 0x02: cmd_parameter(d); return;
    case 0x06: cmd_project(d); return;
    case 0x0e: cmd_target(d); return;
    case 0x0a: raster_line(d, (uint16_t)d->in[0]); return;
    case 0x14: {  /* gyrate: integrate angular velocities (docs sparse; see log) */
      d->out[0] = (int16_t)(d->in[0] + d->in[3]);
      d->out[1] = (int16_t)(d->in[1] + d->in[4]);
      d->out[2] = (int16_t)(d->in[2] + d->in[5]);
      return;
    }
    case 0x0f: case 0x1f: case 0x2f: case 0x3f:
      d->out[0] = 0x0000;      /* self test: pass */
      return;
    default:
      break;
  }
  d->unknownCmds++;
#ifndef TARGET_GNW
  if (d->unknownCmds <= 8)
    fprintf(stderr, "[dsp1] UNKNOWN command %02x\n", c);
#endif
  d->out[0] = 0;
}

/* word counts per command; 0xff = unknown */
static void io_shape(uint8_t cmd, uint8_t* inW, uint8_t* outW) {
  uint8_t c = cmd & 0x3f;          /* dispatch-table index (see execute) */
  int hi = (c >> 4) & 0xf;
  switch (c & 0x0f) {
    case 0x1: if (hi <= 2) { *inW = 4; *outW = 0; return; } break;
    case 0x3: if (hi <= 2) { *inW = 3; *outW = 3; return; } break;
    case 0xb: if (hi <= 2) { *inW = 3; *outW = 1; return; } break;
    case 0xd: if (hi <= 2) { *inW = 3; *outW = 3; return; } break;
    default: break;
  }
  switch (c) {
    case 0x00: *inW = 2; *outW = 1; return;
    case 0x02: *inW = 7; *outW = 4; return;
    case 0x04: *inW = 2; *outW = 2; return;
    case 0x06: *inW = 3; *outW = 3; return;
    case 0x08: *inW = 3; *outW = 2; return;
    case 0x0a: *inW = 1; *outW = 4; return;
    case 0x0c: *inW = 3; *outW = 2; return;
    case 0x0e: *inW = 2; *outW = 2; return;
    case 0x10: *inW = 2; *outW = 2; return;
    case 0x14: *inW = 6; *outW = 3; return;
    case 0x18: *inW = 4; *outW = 1; return;
    case 0x1c: *inW = 6; *outW = 3; return;
    case 0x28: *inW = 3; *outW = 1; return;
    case 0x0f: case 0x1f: case 0x2f: case 0x3f: *inW = 1; *outW = 1; return;
    default:   *inW = 1; *outW = 1; return;   /* unknown: consume one, emit one */
  }
}

/* ---- transfer state machine --------------------------------------------- */

static void start_command(Dsp1* d, uint8_t val) {
#if !defined(NDEBUG_DSP1_TRACE) && !defined(TARGET_GNW)
  static uint64_t seen;            /* one line per distinct opcode bucket, diagnostics */
  if (!(seen & (1ull << (val & 0x3f)))) {
    seen |= 1ull << (val & 0x3f);
    fprintf(stderr, "[dsp1] first use: cmd %02x\n", val);
  }
#endif
  d->cmd = val;
  io_shape(val, &d->inWords, &d->outWords);
  d->byteIdx = 0;
  memset(d->in, 0, sizeof(d->in));
  if (d->inWords == 0) {           /* no params: execute immediately */
    execute(d);
    d->state = d->outWords ? 2 : 0;
  } else {
    d->state = 1;
  }
}

/* byte-level wire trace for bring-up: DSP1_TRACE=1 in the environment.
 * Host-only: getenv() has nothing to read on-device (no environment), so this
 * is compiled out under TARGET_GNW rather than left to always resolve false --
 * it was the sole caller of getenv() in the whole firmware, pulling in real
 * newlib reentrant-environ support for a check that can never do anything. */
#ifndef TARGET_GNW
#include <stdlib.h>
static int trace_on(void) {
  static int t = -1;
  if (t < 0) t = getenv("DSP1_TRACE") ? 1 : 0;
  return t;
}
#else
static int trace_on(void) { return 0; }
#endif

void dsp1_writeDR(Dsp1* d, uint8_t val) {
  if (trace_on()) fprintf(stderr, "W %02x s%d i%d c%02x\n", val, d->state, d->byteIdx, d->cmd);
  switch (d->state) {
    case 0:                        /* idle: command byte */
      start_command(d, val);
      return;
    case 1: {                      /* collecting parameter bytes, LSB first */
      int w = d->byteIdx >> 1;
      if (d->byteIdx & 1) d->in[w] = (int16_t)((d->in[w] & 0x00ff) | (val << 8));
      else                d->in[w] = (int16_t)((d->in[w] & 0xff00) | val);
      d->byteIdx++;
      if (d->byteIdx >= d->inWords * 2) {
        execute(d);
        d->byteIdx = 0;
        if ((d->cmd & 0x3f) == 0x0a) { d->rasterVs = (uint16_t)d->in[0]; d->state = 3; }
        else d->state = d->outWords ? 2 : 0;
      }
      return;
    }
    default:
      /* a write while we are outputting = the host moved on: treat as a new
       * command byte (this is also how the raster stream is terminated) */
      d->state = 0;
      start_command(d, val);
      return;
  }
}

uint8_t dsp1_readDR(Dsp1* d) {
  if (trace_on()) fprintf(stderr, "R s%d i%d c%02x\n", d->state, d->byteIdx, d->cmd);
  switch (d->state) {
    case 2: {                      /* result bytes, LSB first */
      int w = d->byteIdx >> 1;
      uint8_t b = (d->byteIdx & 1) ? (uint8_t)(d->out[w] >> 8) : (uint8_t)d->out[w];
      d->byteIdx++;
      if (d->byteIdx >= d->outWords * 2) { d->state = 0; d->byteIdx = 0; }
      return b;
    }
    case 3: {                      /* raster stream: 4 words per line, then next line */
      int w = d->byteIdx >> 1;
      uint8_t b = (d->byteIdx & 1) ? (uint8_t)(d->out[w] >> 8) : (uint8_t)d->out[w];
      d->byteIdx++;
      if (d->byteIdx >= 8) {       /* line consumed: advance and refill */
        d->byteIdx = 0;
        d->rasterVs++;
        raster_line(d, d->rasterVs);
      }
      return b;
    }
    case 1:
      /* A read while we are collecting parameters means host and chip have
       * lost byte alignment (the boot-time probe spam does this). The host
       * only reads when it believes results are pending, so resynchronize:
       * abandon the partial command; the next write is a command byte. Games
       * re-issue their per-frame command cycle, so one discarded cycle heals
       * everything. */
      d->state = 0;
      d->byteIdx = 0;
      return 0xff;
    default:
      return 0xff;                 /* nothing pending */
  }
}

/* strong allocation pair for cart.c's weak fallbacks */
#include <stdlib.h>
Dsp1* dsp1_alloc(void) { return (Dsp1*)calloc(1, sizeof(Dsp1)); }
uint32_t dsp1_size(void) { return (uint32_t)sizeof(Dsp1); }
