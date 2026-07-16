/* Exact-replay spin-skip learner. See spin_skip.h for the contract; the logic is
 * the harness-proven implementation from tools/snes_spin/skip_harness.c, moved
 * here so the device and the gate harness compile the SAME code. */
#include "spin_skip.h"
#include "cpu.h"

SpinSkip g_spin;

/* Reads within +-6 bytes of the PC are the opcode/operand fetch; WRAM and ROM
 * reads are side-effect-free. Anything else ($21xx APU ports, $42xx HVBJOY/joy,
 * $43xx DMA regs, expansion) observably moves state: an iteration that touches
 * one can terminate on its own and must never be replayed. */
void spin_hook_read(Cpu *cpu, uint32_t adr) {
  if (!g_spin.gate_on) return;
  uint32_t pcb = ((uint32_t)cpu->k << 16) | cpu->pc;
  if (adr - (pcb - 6) <= 12) return;
  uint8_t bank = adr >> 16;
  uint16_t off = (uint16_t)adr;
  bool wram = (bank == 0x7e || bank == 0x7f) ||
              (off < 0x2000 && (bank < 0x40 || (bank >= 0x80 && bank < 0xc0)));
  if (wram) return;
  bool rom = (off >= 0x8000) || (bank >= 0x40 && bank < 0x7e) || (bank >= 0xc0);
  if (rom) return;
  g_spin.io_seq++;
}

/* Register identity is REQUIRED, not optional: a delay loop (`dey / bne`) writes
 * nothing and reads nothing yet terminates on its own — without the regs check
 * it gets adopted and replayed forever (the first harness build did exactly that
 * and died in cart_readLorom). Equal regs at the same PC + no writes + no IO
 * reads = the machine state truly recurred, so the loop provably cannot exit by
 * itself. */
void spin_note(uint32_t pc24, uint8_t charge, int dispatched,
               uint64_t r1, uint64_t r2) {
  SpinSkip *s = &g_spin;
  if (s->on) {
    if (dispatched || pc24 != s->pc[s->idx]) s->on = false;
    else s->idx = (s->idx + 1) % s->len;
  }
  if (!s->gate_on) return;
  s->lr[s->lr_h].pc = pc24; s->lr[s->lr_h].charge = charge;
  s->lr[s->lr_h].w = s->write_seq; s->lr[s->lr_h].io = s->io_seq;
  s->lr[s->lr_h].r1 = r1; s->lr[s->lr_h].r2 = r2;
  s->lr_h = (s->lr_h + 1) % SPIN_LR; if (s->lr_n < SPIN_LR) s->lr_n++;
  if (s->on || dispatched) return;

  for (int d = 1; d <= SPIN_PMAX && 2 * d + 1 <= s->lr_n; d++) {
    int j = (s->lr_h - 1 - d + SPIN_LR) % SPIN_LR;
    if (s->lr[j].pc != pc24) continue;
    /* regs identical at this PC on both prior visits */
    if (s->lr[j].r1 != r1 || s->lr[j].r2 != r2) return;
    /* wseq/ioseq frozen across the last TWO iterations */
    int oldest = (s->lr_h - 1 - 2 * d + SPIN_LR) % SPIN_LR;
    if (s->lr[oldest].w != s->write_seq || s->lr[oldest].io != s->io_seq) return;
    if (s->lr[oldest].pc != pc24) return;
    if (s->lr[oldest].r1 != r1 || s->lr[oldest].r2 != r2) return;
    for (int q = 1; q < d; q++) {
      int a = (s->lr_h - 1 - q + SPIN_LR) % SPIN_LR,
          b = (s->lr_h - 1 - q - d + SPIN_LR) % SPIN_LR;
      if (s->lr[a].pc != s->lr[b].pc || s->lr[a].charge != s->lr[b].charge) return;
    }
    /* adopt: entries [lr_h-d .. lr_h-1] are one iteration ending at pc24;
     * the next opcode to execute is the one that followed the previous pc24 */
    for (int q = 0; q < d; q++) {
      int a = (s->lr_h - d + q + SPIN_LR) % SPIN_LR;
      s->pc[q] = s->lr[a].pc; s->charge[q] = s->lr[a].charge;
    }
    s->len = d; s->idx = 0; s->on = true;
    return;
  }
}

/* Observation-window auto-gate. Window = 600 frames (~10 s). A game that ended
 * the window with <1% of its opcodes replayed is not spinning — park the learner
 * for 1800 frames (~30 s) so it costs nothing, then try again (loading screens
 * end; RPGs start polling once gameplay begins). Replay itself stays armed even
 * while parked: an adopted pattern keeps paying, only LEARNING pauses. */
#define SPIN_WIN_FRAMES  600u
#define SPIN_PARK_FRAMES 1800u

void spin_frame_tick(void) {
  SpinSkip *s = &g_spin;
  if (!s->gate_on) {
    if (s->park_frames > 0 && --s->park_frames == 0) {
      s->gate_on = true;
      s->win_real = 0;
      s->win_virtual = (uint32_t)s->ops_virtual;
    }
    return;
  }
  s->win_real++;   /* frame count doubles as the window clock */
  if (s->win_real < SPIN_WIN_FRAMES) return;
  /* win_virtual snapshots (truncated) ops_virtual at window start; the delta
   * over 600 frames always fits 32 bits (<= ~360M ops even at full tilt). */
  uint32_t replayed = (uint32_t)s->ops_virtual - s->win_virtual;
  s->win_real = 0;
  s->win_virtual = (uint32_t)s->ops_virtual;
  if (replayed < SPIN_WIN_FRAMES * 3u) {   /* < ~3 replayed ops/frame: not a spinner */
    s->gate_on = false;
    s->park_frames = SPIN_PARK_FRAMES;
  }
}

void spin_reset(void) {
  SpinSkip *s = &g_spin;
  *s = (SpinSkip){0};
  s->gate_on = true;
}
