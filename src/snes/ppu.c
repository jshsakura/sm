
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ppu.h"
#include "snes.h"
#include "../types.h"
#ifdef TARGET_GNW
#include "gw_malloc.h"
/* The device framebuffer is RGB565. */
#ifndef PPU_RGB565
#define PPU_RGB565 1
#endif
#endif
typedef uint64_t uint64;
typedef uint32_t uint32;
#ifndef TARGET_GNW
typedef uint32_t uint;
#endif
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint8_t uint8;

extern bool g_new_ppu;
static void PpuDrawWholeLine(Ppu *ppu, uint y);

// array for layer definitions per mode:
//   0-7: mode 0-7; 8: mode 1 + l3prio; 9: mode 7 + extbg

//   0-3; layers 1-4; 4: sprites; 5: nonexistent
static const int layersPerMode[10][12] = {
  {4, 0, 1, 4, 0, 1, 4, 2, 3, 4, 2, 3},
  {4, 0, 1, 4, 0, 1, 4, 2, 4, 2, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5},
  {4, 0, 4, 4, 0, 4, 5, 5, 5, 5, 5, 5},
  {4, 4, 4, 0, 4, 5, 5, 5, 5, 5, 5, 5},
  {2, 4, 0, 1, 4, 0, 1, 4, 4, 2, 5, 5},
  {4, 4, 1, 4, 0, 4, 1, 5, 5, 5, 5, 5}
};

static const int prioritysPerMode[10][12] = {
  {3, 1, 1, 2, 0, 0, 1, 1, 1, 0, 0, 0},
  {3, 1, 1, 2, 0, 0, 1, 1, 0, 0, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5},
  {3, 1, 2, 1, 0, 0, 5, 5, 5, 5, 5, 5},
  {3, 2, 1, 0, 0, 5, 5, 5, 5, 5, 5, 5},
  {1, 3, 1, 1, 2, 0, 0, 1, 0, 0, 5, 5},
  {3, 2, 1, 1, 0, 0, 0, 5, 5, 5, 5, 5}
};

static const int layerCountPerMode[10] = {
  12, 10, 8, 8, 8, 8, 6, 5, 10, 7
};

static const int bitDepthsPerMode[10][4] = {
  {2, 2, 2, 2},
  {4, 4, 2, 5},
  {4, 4, 5, 5},
  {8, 4, 5, 5},
  {8, 2, 5, 5},
  {4, 2, 5, 5},
  {4, 5, 5, 5},
  {8, 5, 5, 5},
  {4, 4, 2, 5},
  {8, 7, 5, 5}
};

static const int spriteSizes[8][2] = {
  {8, 16}, {8, 32}, {8, 64}, {16, 32},
  {16, 64}, {32, 64}, {16, 32}, {16, 32}
};

static void ppu_handlePixel(Ppu* ppu, int x, int y);
static int ppu_getPixel(Ppu* ppu, int x, int y, bool sub, int* r, int* g, int* b);
static uint16_t ppu_getOffsetValue(Ppu* ppu, int col, int row);
static int ppu_getPixelForBgLayer(Ppu* ppu, int x, int y, int layer, bool priority);
static void ppu_handleOPT(Ppu* ppu, int layer, int* lx, int* ly);
static void ppu_calculateMode7Starts(Ppu* ppu, int y);
static int ppu_getPixelForMode7(Ppu* ppu, int x, int layer, bool priority);
static bool ppu_getWindowState(Ppu* ppu, int layer, int x);
static bool ppu_evaluateSprites(Ppu* ppu, int line);
static uint16_t ppu_getVramRemap(Ppu* ppu);
#define SPRITE_PRIO_TO_PRIO(prio, level6) (((prio) * 4 + 2) * 16 + 4 + (level6 ? 2 : 0))
#define SPRITE_PRIO_TO_PRIO_HI(prio) ((prio) * 4 + 2)


#define IS_SCREEN_ENABLED(ppu, sub, layer) (ppu->screenEnabled[sub] & (1 << layer))
#define IS_SCREEN_WINDOWED(ppu, sub, layer) (ppu->screenWindowed[sub] & (1 << layer))
#define IS_MOSAIC_ENABLED(ppu, layer) ((ppu->mosaicEnabled & (1 << layer)))
#define GET_WINDOW_FLAGS(ppu, layer) (ppu->windowsel >> (layer * 4))
enum {
  kWindow1Inversed = 1,
  kWindow1Enabled = 2,
  kWindow2Inversed = 4,
  kWindow2Enabled = 8,
};

Ppu* ppu_init(Snes* snes) {
#ifdef TARGET_GNW
  /* One PPU on the device (there is no reference emulator to run alongside), so
   * a static instance beats a malloc from the overlay pool. */
  static Ppu g_ppu;
  Ppu* ppu = &g_ppu;
#if defined(GNW_SNES_CORE)
  /* SNES overlay only: VRAM in overlay BSS (RAM_EMU) as a static array — frees
   * 64 KB of ITCM for the rc hot subset. The GNW_SNES_CORE guard keeps this
   * array out of the SM overlay (which also compiles ppu.o for shared symbols
   * but has its own PPU and its own tight BSS budget). */
  static uint16_t g_ppu_vram[0x8000];
  if (ppu->vram == NULL)
    ppu->vram = g_ppu_vram;
#else
  /* Other GNW overlays (SM etc.): VRAM in ITCM as before. */
  if (ppu->vram == NULL)
    ppu->vram = (uint16_t *)itc_calloc(1, 0x10000);
#endif
#else
  Ppu* ppu = malloc(sizeof(Ppu));
#endif
  ppu->snes = snes;
  return ppu;
}

void ppu_free(Ppu* ppu) {
#ifndef TARGET_GNW
  free(ppu);
#endif
}

#ifndef TARGET_GNW
void ppu_copy(Ppu *ppu, Ppu *ppu_src) {
  Snes *snes = ppu->snes;
  size_t pitch = ppu->renderPitch;
  uint8_t *renderBuffer = ppu->renderBuffer;
  memcpy(ppu, ppu_src, sizeof(*ppu));
  ppu->renderBuffer = renderBuffer;
  ppu->renderPitch = (uint32_t)pitch;
  ppu->snes = snes;
}
#endif  /* !TARGET_GNW */

void ppu_reset(Ppu* ppu) {
  {
    Snes *snes = ppu->snes;
    size_t pitch = ppu->renderPitch;
    uint8_t *renderBuffer = ppu->renderBuffer;
#ifdef TARGET_GNW
    /* vram is a pointer now: the wholesale memset below would throw it away. */
    uint16_t *vram = ppu->vram;
#endif
    memset(ppu, 0, sizeof(*ppu));
#ifdef TARGET_GNW
    ppu->vram = vram;
    memset(ppu->vram, 0, 0x10000);
#endif
    ppu->renderBuffer = renderBuffer;
    ppu->renderPitch = (uint32_t)pitch;
    ppu->snes = snes;
  }
  ppu->vramPointer = 0;
  ppu->vramIncrementOnHigh = false;
  ppu->vramIncrement = 1;
  ppu->vramRemapMode = 0;
  ppu->vramReadBuffer = 0;
  memset(ppu->cgram, 0, sizeof(ppu->cgram));
  ppu->cgramPointer = 0;
  ppu->cgramSecondWrite = false;
  ppu->cgramBuffer = 0;
  memset(ppu->oam, 0, sizeof(ppu->oam));
  memset(ppu->highOam, 0, sizeof(ppu->highOam));
  ppu->oamAdr = 0;
  ppu->oamAdrWritten = 0;
  ppu->oamInHigh = false;
  ppu->oamInHighWritten = false;
  ppu->oamSecondWrite = false;
  ppu->oamBuffer = 0;
  ppu->objPriority = false;
  ppu->objTileAdr1 = 0;
  ppu->objTileAdr2 = 0;
  ppu->objSize = 0;
  ppu->timeOver = false;
  ppu->rangeOver = false;
  ppu->objInterlace = false;
  for(int i = 0; i < 4; i++) {
    ppu->bgLayer[i].hScroll = 0;
    ppu->bgLayer[i].vScroll = 0;
    ppu->bgLayer[i].tilemapWider = false;
    ppu->bgLayer[i].tilemapHigher = false;
    ppu->bgLayer[i].tilemapAdr = 0;
    ppu->bgLayer[i].tileAdr = 0;
    ppu->bgLayer[i].bigTiles = false;
    ppu->bgLayer[i].mosaicEnabled = false;
  }
  ppu->scrollPrev = 0;
  ppu->scrollPrev2 = 0;
  ppu->mosaicSize = 1;
  ppu->mosaicStartLine = 1;
  for(int i = 0; i < 5; i++) {
    ppu->layer[i].mainScreenEnabled = false;
    ppu->layer[i].subScreenEnabled = false;
    ppu->layer[i].mainScreenWindowed = false;
    ppu->layer[i].subScreenWindowed = false;
  }
  memset(ppu->m7matrix, 0, sizeof(ppu->m7matrix));
  ppu->m7prev = 0;
  ppu->m7largeField = false;
  ppu->m7charFill = false;
  ppu->m7xFlip = false;
  ppu->m7yFlip = false;
  ppu->m7extBg = false;
  ppu->m7startX = 0;
  ppu->m7startY = 0;
  for(int i = 0; i < 6; i++) {
    ppu->windowLayer[i].window1enabled = false;
    ppu->windowLayer[i].window2enabled = false;
    ppu->windowLayer[i].window1inversed = false;
    ppu->windowLayer[i].window2inversed = false;
    ppu->windowLayer[i].maskLogic = 0;
  }
  ppu->window1left = 0;
  ppu->window1right = 0;
  ppu->window2left = 0;
  ppu->window2right = 0;
  ppu->clipMode = 0;
  ppu->preventMathMode = 0;
  ppu->addSubscreen = false;
  ppu->subtractColor = false;
  ppu->halfColor = false;
  memset(ppu->mathEnabled, 0, sizeof(ppu->mathEnabled));
  ppu->fixedColorR = 0;
  ppu->fixedColorG = 0;
  ppu->fixedColorB = 0;
  ppu->forcedBlank = true;
  ppu->brightness = 0;
  ppu->mode = 0;
  ppu->bg3priority = false;
  ppu->evenFrame = false;
  ppu->pseudoHires = false;
  ppu->overscan = false;
  ppu->frameOverscan = false;
  ppu->interlace = false;
  ppu->frameInterlace = false;
  ppu->directColor = false;
  ppu->hCount = 0;
  ppu->vCount = 0;
  ppu->hCountSecond = false;
  ppu->vCountSecond = false;
  ppu->countersLatched = false;
  ppu->ppu1openBus = 0;
  ppu->ppu2openBus = 0;
#ifdef SNES_LINE_CACHE
  ppu_lineCacheInvalidate();
#endif
}

/* ppu_write() stores every screen-enable and window register TWICE: unpacked into
 * layer[]/windowLayer[], and packed into screenEnabled/screenWindowed/windowsel —
 * the byte the game actually wrote. The renderer reads only the packed copies:
 *
 *     #define IS_SCREEN_ENABLED(ppu, sub, layer) (ppu->screenEnabled[sub] & (1 << layer))
 *
 * and the packed copies sit past pixelbuffer_placeholder, so the savestate does
 * not carry them. They are caches, exactly like palette565 — and like palette565
 * a load has to rebuild them, because the unpacked originals it DID restore are
 * not what anything looks at.
 *
 * Left alone, screenEnabled stays whatever it was. Load into a PPU that was just
 * ppu_reset() — which is every "resume from a savestate" on the G&W, because the
 * launcher boots the core and loads second — and it is zero: no BG, no sprites,
 * every line composited as bare backdrop. A black screen that still runs at full
 * speed on almost no CPU, because there is nothing left to draw. */
static void ppu_rebuild_packed_registers(Ppu *ppu) {
  uint8_t tm = 0, ts = 0, tmw = 0, tsw = 0;

  for (int i = 0; i < 5; i++) {   /* BG1..BG4, OBJ — $212C/$212D/$212E/$212F */
    if (ppu->layer[i].mainScreenEnabled)  tm  |= 1 << i;
    if (ppu->layer[i].subScreenEnabled)   ts  |= 1 << i;
    if (ppu->layer[i].mainScreenWindowed) tmw |= 1 << i;
    if (ppu->layer[i].subScreenWindowed)  tsw |= 1 << i;
  }
  ppu->screenEnabled[0] = tm;
  ppu->screenEnabled[1] = ts;
  ppu->screenWindowed[0] = tmw;
  ppu->screenWindowed[1] = tsw;

  /* windowsel is six 4-bit fields, one per layer, in the order GET_WINDOW_FLAGS
   * indexes them — the same nibble ppu_write() packs from $2123..$2125. */
  uint32_t sel = 0;
  for (int i = 0; i < 6; i++) {
    uint32_t flags = 0;
    if (ppu->windowLayer[i].window1inversed) flags |= kWindow1Inversed;
    if (ppu->windowLayer[i].window1enabled)  flags |= kWindow1Enabled;
    if (ppu->windowLayer[i].window2inversed) flags |= kWindow2Inversed;
    if (ppu->windowLayer[i].window2enabled)  flags |= kWindow2Enabled;
    sel |= flags << (i * 4);
  }
  ppu->windowsel = sel;
}

void ppu_saveload(Ppu *ppu, SaveLoadFunc *func, void *ctx) {
#ifdef PPU_RGB565
  /* Everything the PPU derives from cgram and brightness lives outside the saved
   * region — the RGB565 palette and the brightness table are caches, not state.
   * A load restores cgram underneath them and nothing tells them so: the screen
   * then draws the scene you loaded with the colours of the scene you left, or,
   * on a PPU that has drawn nothing yet, with no colours at all. Invalidate them.
   * (0xff is not a brightness, so the table rebuilds on the next line.) */
  ppu->paletteDirty = true;
  ppu->lastBrightnessMult = 0xff;
#endif
  /* Same rule for the sprite cache: a load restores OAM underneath it. And the
   * objBuffer contents aren't part of the stream either, so stop trusting them. */
  ppu->objCacheValid = 0;
  ppu->objBufferClean = 0;
#ifdef TARGET_GNW
  /* vram lives in ITC RAM now, so it is no longer contiguous with the rest of
   * the struct. Emit the identical byte stream — VRAM first, then everything
   * from vramPointer on — so savestates stay compatible with the PC build. */
  func(ctx, ppu->vram, 0x8000 * sizeof(uint16_t));
  func(ctx, &ppu->vramPointer,
       offsetof(Ppu, pixelbuffer_placeholder) - offsetof(Ppu, vramPointer));
#else
  func(ctx, &ppu->vram, offsetof(Ppu, pixelbuffer_placeholder) - offsetof(Ppu, vram));
#endif

  /* After the stream, so a load rebuilds from what it just read. On a save this
   * recomputes the values it already had — the two copies agree by construction,
   * ppu_write() writes both — so it is a no-op there rather than a special case. */
  ppu_rebuild_packed_registers(ppu);
#ifdef SNES_LINE_CACHE
  /* The framebuffer is not part of the savestate stream.  Whether this call
   * saved or loaded, stop trusting its pixels until every line is redrawn. */
  ppu_lineCacheInvalidate();
#endif
}

void PpuBeginDrawing(Ppu *ppu, uint8_t *pixels, size_t pitch, uint32_t render_flags) {
  ppu->renderPitch = (uint)pitch;
  ppu->renderBuffer = pixels;
}

bool ppu_checkOverscan(Ppu* ppu) {
  // called at (0,225)
  ppu->frameOverscan = ppu->overscan; // set if we have a overscan-frame
  return ppu->frameOverscan;
}

void ppu_handleVblank(Ppu* ppu) {
  // called either right after ppu_checkOverscan at (0,225), or at (0,240)
  if(!ppu->forcedBlank) {
    ppu->oamAdr = ppu->oamAdrWritten;
    ppu->oamInHigh = ppu->oamInHighWritten;
    ppu->oamSecondWrite = false;
  }
  ppu->frameInterlace = ppu->interlace; // set if we have a interlaced frame
}

bool g_ppu_skip_render;

#ifdef TARGET_GNW
void (*g_ppu_line_cb)(unsigned y, const uint16_t *line);
#endif

_Static_assert(_Alignof(PpuPixelPrioBufs) >= 8,
               "ClearBackdrop writes 64 bits at a time; on ARM that is STRD, which "
               "faults on an unaligned address. Keep the aligned(8) on the struct.");

#if defined(SNES_LINE_REUSE_PROBE) || defined(SNES_LINE_CACHE)
enum { kLineHistoryLines = 240, kLineHistoryVramPages = 512 };
#endif

static inline void ClearBackdrop(PpuPixelPrioBufs *buf) {
  for (size_t i = 0; i != arraysize(buf->data); i += 4)
    *(uint64*)&buf->data[i] = 0x0500050005000500;
}

#ifdef SNES_LINE_REUSE_PROBE
/* Observation only: predict from inputs, still render, then compare the exact
 * RGB565 line against the previous frame. None of this state is emulated. */
typedef struct PpuLineProbeState {
  BgLayer bgLayer[4];
  int16_t m7matrix[8];
  uint32_t windowsel;
  uint16_t objTileAdr1, objTileAdr2;
  uint8_t objPriority, objSize, objInterlace, oamAdr;
  uint8_t mosaicSize, mosaicStartLine, mosaicEnabled;
  uint8_t window1left, window1right, window2left, window2right;
  uint8_t clipMode, preventMathMode, addSubscreen, subtractColor, halfColor;
  uint8_t mathEnabled[6];
  uint8_t fixedColorR, fixedColorG, fixedColorB;
  uint8_t forcedBlank, brightness, mode, bg3priority;
  uint8_t pseudoHires, directColor, m7largeField, m7charFill, m7xFlip, m7yFlip, m7extBg;
  uint8_t screenEnabled[2], screenWindowed[2];
  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t lineHasSprites, evenFrameWhenObjInterlace;
} PpuLineProbeState;

typedef struct PpuLineProbeStats {
  uint64_t total, actualSame, predicted, falsePositive, falseNegative;
} PpuLineProbeStats;

enum { kProbeLines = kLineHistoryLines, kProbeBuckets = 4, kProbeVariants = 12,
       kProbeVramPages = kLineHistoryVramPages };
static PpuLineProbeState g_probe_prev_state[kProbeLines];
static uint32_t g_probe_prev_vram[kProbeLines], g_probe_prev_cgram[kProbeLines], g_probe_prev_oam[kProbeLines];
static uint8_t g_probe_prev_line[kProbeLines][kPpuXPixels * sizeof(uint16_t)];
static uint8_t g_probe_valid[kProbeLines];
static uint8_t g_probe_pending[kProbeVariants];
static uint32_t g_probe_vram_gen, g_probe_cgram_gen, g_probe_oam_gen;
static uint32_t g_probe_vram_page_gen[kProbeVramPages], g_probe_oam_entry_gen[128];
static uint32_t g_probe_cgram_entry_gen[256];
static uint32_t g_probe_prev_vram_page_gen[kProbeLines][kProbeVramPages];
static uint32_t g_probe_prev_oam_entry_gen[kProbeLines][128];
static uint32_t g_probe_prev_cgram_entry_gen[kProbeLines][256];
static uint32_t g_probe_prev_vram_mask[kProbeLines][16], g_probe_cur_vram_mask[16];
static uint32_t g_probe_prev_oam_mask[kProbeLines][4];
static uint32_t g_probe_prev_cgram_mask[kProbeLines][8];
static uint32_t g_probe_frame;
static PpuLineProbeStats g_probe_stats[kProbeBuckets + 1][kProbeVariants];

static inline void PpuLineProbeVram(uint32_t adr) {
  g_probe_cur_vram_mask[(adr >> 11) & 15] |= 1u << ((adr >> 6) & 31);
}

static inline uint16_t PpuLineProbeVramPtr(Ppu *ppu, const uint16_t *ptr) {
  PpuLineProbeVram((uint32_t)(ptr - ppu->vram) & 0x7fff);
  return *ptr;
}

static void PpuLineProbeCapture(PpuLineProbeState *s, const Ppu *ppu) {
  memset(s, 0, sizeof(*s));
  memcpy(s->bgLayer, ppu->bgLayer, sizeof(s->bgLayer));
  memcpy(s->m7matrix, ppu->m7matrix, sizeof(s->m7matrix));
  memcpy(s->mathEnabled, ppu->mathEnabled, sizeof(s->mathEnabled));
  memcpy(s->screenEnabled, ppu->screenEnabled, sizeof(s->screenEnabled));
  memcpy(s->screenWindowed, ppu->screenWindowed, sizeof(s->screenWindowed));
  s->windowsel = ppu->windowsel;
  s->objTileAdr1 = ppu->objTileAdr1; s->objTileAdr2 = ppu->objTileAdr2;
  s->objPriority = ppu->objPriority; s->objSize = ppu->objSize;
  s->objInterlace = ppu->objInterlace; s->oamAdr = ppu->oamAdr;
  s->mosaicSize = ppu->mosaicSize; s->mosaicStartLine = ppu->mosaicStartLine;
  s->mosaicEnabled = ppu->mosaicEnabled;
  s->window1left = ppu->window1left; s->window1right = ppu->window1right;
  s->window2left = ppu->window2left; s->window2right = ppu->window2right;
  s->clipMode = ppu->clipMode; s->preventMathMode = ppu->preventMathMode;
  s->addSubscreen = ppu->addSubscreen; s->subtractColor = ppu->subtractColor;
  s->halfColor = ppu->halfColor;
  s->fixedColorR = ppu->fixedColorR; s->fixedColorG = ppu->fixedColorG;
  s->fixedColorB = ppu->fixedColorB;
  s->forcedBlank = ppu->forcedBlank; s->brightness = ppu->brightness;
  s->mode = ppu->mode; s->bg3priority = ppu->bg3priority;
  s->pseudoHires = ppu->pseudoHires; s->directColor = ppu->directColor;
  s->m7largeField = ppu->m7largeField; s->m7charFill = ppu->m7charFill;
  s->m7xFlip = ppu->m7xFlip; s->m7yFlip = ppu->m7yFlip; s->m7extBg = ppu->m7extBg;
  s->extraLeftCur = ppu->extraLeftCur; s->extraRightCur = ppu->extraRightCur;
  s->extraLeftRight = ppu->extraLeftRight; s->lineHasSprites = ppu->lineHasSprites;
  s->evenFrameWhenObjInterlace = ppu->objInterlace ? ppu->evenFrame : 0;
}

static void PpuLineProbeFieldDiff(const PpuLineProbeState *cur, const PpuLineProbeState *prev, uint32_t bucket);

static bool PpuLineProbeRegsMatchExcl(const PpuLineProbeState *a, const PpuLineProbeState *b) {
  const uint8_t *pa = (const uint8_t *)a, *pb = (const uint8_t *)b;
  size_t sz = sizeof(PpuLineProbeState);
  bool skip_m7 = (a->mode != 7 && b->mode != 7);
  size_t m7_off = offsetof(PpuLineProbeState, m7matrix);
  size_t m7_end = m7_off + sizeof(a->m7matrix);
  size_t oam_off = offsetof(PpuLineProbeState, oamAdr);
  for (size_t i = 0; i < sz; i++) {
    if (i == oam_off) continue;
    if (skip_m7 && i >= m7_off && i < m7_end) continue;
    if (pa[i] != pb[i]) return false;
  }
  return true;
}

static void PpuLineProbeBefore(Ppu *ppu, int line) {
  PpuLineProbeState cur;
  PpuLineProbeCapture(&cur, ppu);
  int y = line - 1;
  bool regs_raw = g_probe_valid[y] && memcmp(&cur, &g_probe_prev_state[y], sizeof(cur)) == 0;
  bool regs = regs_raw;
  if (g_probe_valid[y] && !regs_raw) {
    uint32_t bucket = (g_probe_frame - 1) / 300;
    if (bucket >= kProbeBuckets) bucket = kProbeBuckets - 1;
    PpuLineProbeFieldDiff(&cur, &g_probe_prev_state[y], bucket);
  }
  bool vr = g_probe_vram_gen == g_probe_prev_vram[y];
  bool cg = g_probe_cgram_gen == g_probe_prev_cgram[y];
  bool oa = g_probe_oam_gen == g_probe_prev_oam[y];
  bool vr_pages = g_probe_valid[y];
  for (int page = 0; page < kProbeVramPages && vr_pages; page++)
    if ((g_probe_prev_vram_mask[y][page >> 5] & (1u << (page & 31))) &&
        g_probe_vram_page_gen[page] != g_probe_prev_vram_page_gen[y][page])
      vr_pages = false;
  const uint32_t *cur_oam_mask = ppu->objLineCand[y];
  bool oa_line = g_probe_valid[y];
  for (int s = 0; s < 128 && oa_line; s++)
    if (((cur_oam_mask[s >> 5] | g_probe_prev_oam_mask[y][s >> 5]) & (1u << (s & 31))) &&
        g_probe_oam_entry_gen[s] != g_probe_prev_oam_entry_gen[y][s])
      oa_line = false;
  bool cg_line = g_probe_valid[y];
  for (int index = 0; index < 256 && cg_line; index++)
    if ((g_probe_prev_cgram_mask[y][index >> 5] & (1u << (index & 31))) &&
        g_probe_cgram_entry_gen[index] != g_probe_prev_cgram_entry_gen[y][index])
      cg_line = false;
  g_probe_pending[0] = regs && vr && cg && oa; /* conservative */
  g_probe_pending[1] = regs && vr && cg;       /* omit OAM */
  g_probe_pending[2] = regs && cg && oa;       /* omit VRAM */
  g_probe_pending[3] = regs && vr && oa;       /* omit CGRAM */
  g_probe_pending[4] = regs;                   /* registers only */
  g_probe_pending[5] = regs && vr_pages && cg && oa;
  g_probe_pending[6] = regs && vr_pages && cg && oa_line;
  g_probe_pending[7] = regs && vr_pages && cg_line && oa_line;
  g_probe_pending[8] = regs && vr_pages && cg_line && (!ppu->lineHasSprites || oa_line);
  bool regs_excl = g_probe_valid[y] && PpuLineProbeRegsMatchExcl(&cur, &g_probe_prev_state[y]);
  g_probe_pending[9] = regs_excl && vr_pages && cg && oa_line;
  g_probe_pending[10] = regs_excl && vr_pages && cg_line && oa_line;
  g_probe_pending[11] = regs_excl && vr_pages && cg_line && (!ppu->lineHasSprites || oa_line);
  g_probe_prev_state[y] = cur;
  g_probe_prev_vram[y] = g_probe_vram_gen;
  g_probe_prev_cgram[y] = g_probe_cgram_gen;
  g_probe_prev_oam[y] = g_probe_oam_gen;
  memcpy(g_probe_prev_vram_page_gen[y], g_probe_vram_page_gen, sizeof(g_probe_vram_page_gen));
  memcpy(g_probe_prev_oam_entry_gen[y], g_probe_oam_entry_gen, sizeof(g_probe_oam_entry_gen));
  memcpy(g_probe_prev_cgram_entry_gen[y], g_probe_cgram_entry_gen, sizeof(g_probe_cgram_entry_gen));
  memcpy(g_probe_prev_oam_mask[y], cur_oam_mask, sizeof(g_probe_prev_oam_mask[y]));
}

static void PpuLineProbeAfter(Ppu *ppu, int line) {
  int y = line - 1;
  const uint8_t *cur = ppu->renderBuffer + y * ppu->renderPitch;
  bool same = g_probe_valid[y] && memcmp(cur, g_probe_prev_line[y], sizeof(g_probe_prev_line[y])) == 0;
  if (g_probe_valid[y]) {
    uint32_t bucket = (g_probe_frame - 1) / 300;
    if (bucket >= kProbeBuckets) bucket = kProbeBuckets - 1;
    for (int v = 0; v < kProbeVariants; v++) {
      PpuLineProbeStats *all = &g_probe_stats[kProbeBuckets][v];
      PpuLineProbeStats *part = &g_probe_stats[bucket][v];
#define ADD_STAT(field, value) do { all->field += (value); part->field += (value); } while (0)
      ADD_STAT(total, 1);
      ADD_STAT(actualSame, same);
      ADD_STAT(predicted, g_probe_pending[v]);
      ADD_STAT(falsePositive, g_probe_pending[v] && !same);
      ADD_STAT(falseNegative, !g_probe_pending[v] && same);
#undef ADD_STAT
    }
  }
  memcpy(g_probe_prev_line[y], cur, sizeof(g_probe_prev_line[y]));
  memset(g_probe_prev_cgram_mask[y], 0, sizeof(g_probe_prev_cgram_mask[y]));
  uint32_t math_enabled = 0;
  for (int layer = 0; layer < 6; layer++) math_enabled |= ppu->mathEnabled[layer] << layer;
  bool uses_subscreen = ppu->preventMathMode != 3 && ppu->addSubscreen &&
      math_enabled && ppu->screenEnabled[1] != 0;
  for (int x = 0; x < kPpuXPixels; x++) {
    uint8_t main_index = ppu->bgBuffers[0].data[x] & 0xff;
    g_probe_prev_cgram_mask[y][main_index >> 5] |= 1u << (main_index & 31);
    if (uses_subscreen) {
      uint8_t sub_index = ppu->bgBuffers[1].data[x] & 0xff;
      g_probe_prev_cgram_mask[y][sub_index >> 5] |= 1u << (sub_index & 31);
    }
  }
  memcpy(g_probe_prev_vram_mask[y], g_probe_cur_vram_mask, sizeof(g_probe_cur_vram_mask));
  g_probe_valid[y] = 1;
}

void ppu_lineReuseProbeReport(void) {
  static const char *const names[kProbeVariants] = {
    "full", "no_oam", "no_vram", "no_cgram", "regs", "vram_pages", "vram_pages_oam_line",
    "pages_oam_cgram_line", "pages_cgram_no_sprite",
    "excl_oamadr_pages", "excl_oamadr_cgram_line", "excl_oamadr_no_sprite"
  };
  for (int b = 0; b <= kProbeBuckets; b++) {
    for (int v = 0; v < kProbeVariants; v++) {
      const PpuLineProbeStats *s = &g_probe_stats[b][v];
      printf("[line-reuse] bucket=%s variant=%s total=%llu actual=%llu predicted=%llu fp=%llu fn=%llu pred_x10000=%llu fn_x10000=%llu\n",
          b == kProbeBuckets ? "all" : (b == 0 ? "0-299" : b == 1 ? "300-599" : b == 2 ? "600-899" : "900-1199"), names[v],
          (unsigned long long)s->total, (unsigned long long)s->actualSame,
          (unsigned long long)s->predicted, (unsigned long long)s->falsePositive,
          (unsigned long long)s->falseNegative,
          (unsigned long long)(s->total ? s->predicted * 10000 / s->total : 0),
          (unsigned long long)(s->actualSame ? s->falseNegative * 10000 / s->actualSame : 0));
    }
  }
}

/* Field-level diff tracker: when regs memcmp fails, identify which field(s) differ. */
static uint32_t g_probe_fdiff[kProbeBuckets + 1][sizeof(PpuLineProbeState)];
static uint64_t g_probe_fdiff_calls[kProbeBuckets + 1];

static void PpuLineProbeFieldDiff(const PpuLineProbeState *cur, const PpuLineProbeState *prev, uint32_t bucket) {
  const uint8_t *pa = (const uint8_t *)cur, *pb = (const uint8_t *)prev;
  g_probe_fdiff_calls[bucket]++;
  g_probe_fdiff_calls[kProbeBuckets]++;
  for (size_t i = 0; i < sizeof(PpuLineProbeState); i++)
    if (pa[i] != pb[i]) {
      g_probe_fdiff[bucket][i]++;
      g_probe_fdiff[kProbeBuckets][i]++;
    }
}

/* Field name lookup for reporting */
static const char *ppuProbeFieldName(size_t off) {
#define OFF(field) offsetof(PpuLineProbeState, field)
  if (off < OFF(m7matrix))              return "bgLayer";
  if (off < OFF(windowsel))             return "m7matrix";
  if (off < OFF(objTileAdr1))           return "windowsel";
  if (off < OFF(objPriority))           return "objTileAdr";
  if (off < OFF(mosaicSize))            return "objCfg(oamAdr etc)";
  if (off < OFF(window1left))           return "mosaic";
  if (off < OFF(clipMode))              return "window";
  if (off < OFF(mathEnabled))           return "clipMath(addSub/sub/half)";
  if (off < OFF(fixedColorR))           return "mathEnabled";
  if (off < OFF(forcedBlank))           return "fixedColor";
  if (off < OFF(pseudoHires))           return "blankBrightMode";
  if (off < OFF(screenEnabled))         return "bg3prio/hires/direct/m7opts";
  if (off < OFF(extraLeftCur))          return "screen";
  if (off < OFF(lineHasSprites))        return "extraLeftRight";
  if (off < sizeof(PpuLineProbeState))  return "sprites/evenFrame";
  return "?";
#undef OFF
}

void ppu_lineProbeFieldReport(void) {
  for (int b = 0; b <= kProbeBuckets; b++) {
    const char *bname = b == kProbeBuckets ? "all" : (b == 0 ? "0-299" : b == 1 ? "300-599" : b == 2 ? "600-899" : "900-1199");
    /* Aggregate by field name, not byte offset, for readability */
    uint64_t agg[20]; memset(agg, 0, sizeof(agg));
    const char *names[20];
    int nfields = 0;
    for (size_t i = 0; i < sizeof(PpuLineProbeState); i++) {
      if (g_probe_fdiff[b][i] == 0) continue;
      const char *fn = ppuProbeFieldName(i);
      int idx = -1;
      for (int j = 0; j < nfields; j++) if (names[j] == fn) { idx = j; break; }
      if (idx < 0) { idx = nfields++; names[idx] = fn; }
      agg[idx] += g_probe_fdiff[b][i];
    }
    uint64_t calls = g_probe_fdiff_calls[b];
    printf("[field-diff] bucket=%s regs_false_calls=%llu\n", bname, (unsigned long long)calls);
    for (int j = 0; j < nfields; j++)
      printf("[field-diff]   field=%-24s changes=%llu (%llu%% of regs_false)\n",
             names[j], (unsigned long long)agg[j],
             (unsigned long long)(calls ? agg[j] * 100 / calls : 0));
  }
  /* Also dump raw byte offsets for top noisy fields in gameplay bucket */
  int b = 3; /* 900-1199 */
  printf("[field-diff] raw byte offsets (bucket 900-1199, top 20):\n");
  uint32_t sorted_idx[sizeof(PpuLineProbeState)];
  for (size_t i = 0; i < sizeof(PpuLineProbeState); i++) sorted_idx[i] = (uint32_t)i;
  /* simple insertion sort by count desc */
  for (size_t i = 1; i < sizeof(PpuLineProbeState); i++) {
    for (size_t j = i; j > 0 && g_probe_fdiff[b][sorted_idx[j]] > g_probe_fdiff[b][sorted_idx[j-1]]; j--) {
      uint32_t tmp = sorted_idx[j]; sorted_idx[j] = sorted_idx[j-1]; sorted_idx[j-1] = tmp;
    }
  }
  int shown = 0;
  for (size_t i = 0; i < sizeof(PpuLineProbeState) && shown < 20; i++) {
    uint32_t off = sorted_idx[i];
    if (g_probe_fdiff[b][off] == 0) break;
    printf("[field-diff]   offset=%3u count=%-8u field=%s\n",
           off, g_probe_fdiff[b][off], ppuProbeFieldName(off));
    shown++;
  }
}
#endif

#ifdef SNES_LINE_CACHE
typedef struct PpuLineCacheState {
  BgLayer bgLayer[4];
  int16_t m7matrix[8];
  uint32_t windowsel;
  uint16_t objTileAdr1, objTileAdr2;
  uint8_t objPriority, objSize, objInterlace, oamAdr;
  uint8_t mosaicSize, mosaicStartLine, mosaicEnabled;
  uint8_t window1left, window1right, window2left, window2right;
  uint8_t clipMode, preventMathMode, addSubscreen, subtractColor, halfColor;
  uint8_t mathEnabled[6];
  uint8_t fixedColorR, fixedColorG, fixedColorB;
  uint8_t forcedBlank, brightness, mode, bg3priority;
  uint8_t pseudoHires, directColor, m7largeField, m7charFill, m7xFlip, m7yFlip, m7extBg;
  uint8_t screenEnabled[2], screenWindowed[2];
  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t lineHasSprites, evenFrameWhenObjInterlace;
} PpuLineCacheState;

enum { kLineCacheBuckets = 4, kLineCacheVramShift = 8,
       kLineCacheVramPages = 0x8000 >> kLineCacheVramShift,
       kLineCacheVramWords = kLineCacheVramPages / 32 };
typedef struct PpuLineCacheStats {
  uint32_t total, hits;
} PpuLineCacheStats;

static PpuLineCacheState g_line_cache_state[kLineHistoryLines], g_line_cache_current;
static uint32_t g_line_cache_vram_dep[kLineHistoryLines][kLineCacheVramWords], g_line_cache_cgram_dep[kLineHistoryLines][8];
static uint32_t g_line_cache_oam_dep[kLineHistoryLines][4], g_line_cache_cur_vram[kLineCacheVramWords];
static uint32_t g_line_cache_vram_last[kLineCacheVramPages], g_line_cache_cgram_last[256];
static uint32_t g_line_cache_oam_last[128];
static uint32_t g_line_cache_vram_serial, g_line_cache_cgram_serial, g_line_cache_oam_serial;
static uint32_t g_line_cache_vram_at[kLineHistoryLines], g_line_cache_cgram_at[kLineHistoryLines];
static uint32_t g_line_cache_oam_at[kLineHistoryLines], g_line_cache_frame;
static uint8_t g_line_cache_valid[kLineHistoryLines];
static uint8_t g_line_cache_cooldown[kLineHistoryLines];
static bool g_line_cache_tracking;
static PpuLineCacheStats g_line_cache_stats[kLineCacheBuckets + 1];

static inline bool PpuLineCacheBeginLine(int y) {
  if (g_line_cache_cooldown[y]) {
    g_line_cache_cooldown[y]--;
    return false;
  }
  return true;
}

static inline void PpuLineCacheMiss(int y) {
  g_line_cache_valid[y] = 0;
}

static void PpuLineCacheCapture(PpuLineCacheState *s, const Ppu *ppu) {
  memset(s, 0, sizeof(*s));
  memcpy(s->bgLayer, ppu->bgLayer, sizeof(s->bgLayer));
  memcpy(s->m7matrix, ppu->m7matrix, sizeof(s->m7matrix));
  memcpy(s->mathEnabled, ppu->mathEnabled, sizeof(s->mathEnabled));
  memcpy(s->screenEnabled, ppu->screenEnabled, sizeof(s->screenEnabled));
  memcpy(s->screenWindowed, ppu->screenWindowed, sizeof(s->screenWindowed));
  s->windowsel = ppu->windowsel;
  s->objTileAdr1 = ppu->objTileAdr1; s->objTileAdr2 = ppu->objTileAdr2;
  s->objPriority = ppu->objPriority; s->objSize = ppu->objSize;
  s->objInterlace = ppu->objInterlace; s->oamAdr = ppu->oamAdr;
  s->mosaicSize = ppu->mosaicSize; s->mosaicStartLine = ppu->mosaicStartLine;
  s->mosaicEnabled = ppu->mosaicEnabled;
  s->window1left = ppu->window1left; s->window1right = ppu->window1right;
  s->window2left = ppu->window2left; s->window2right = ppu->window2right;
  s->clipMode = ppu->clipMode; s->preventMathMode = ppu->preventMathMode;
  s->addSubscreen = ppu->addSubscreen; s->subtractColor = ppu->subtractColor;
  s->halfColor = ppu->halfColor;
  s->fixedColorR = ppu->fixedColorR; s->fixedColorG = ppu->fixedColorG;
  s->fixedColorB = ppu->fixedColorB;
  s->forcedBlank = ppu->forcedBlank; s->brightness = ppu->brightness;
  s->mode = ppu->mode; s->bg3priority = ppu->bg3priority;
  s->pseudoHires = ppu->pseudoHires; s->directColor = ppu->directColor;
  s->m7largeField = ppu->m7largeField; s->m7charFill = ppu->m7charFill;
  s->m7xFlip = ppu->m7xFlip; s->m7yFlip = ppu->m7yFlip; s->m7extBg = ppu->m7extBg;
  s->extraLeftCur = ppu->extraLeftCur; s->extraRightCur = ppu->extraRightCur;
  s->extraLeftRight = ppu->extraLeftRight; s->lineHasSprites = ppu->lineHasSprites;
  s->evenFrameWhenObjInterlace = ppu->objInterlace ? ppu->evenFrame : 0;
}

static inline void PpuLineCacheBump(uint32_t *serial, uint32_t *last) {
  uint32_t next = *serial + 1;
  if (next == 0) {
    memset(g_line_cache_valid, 0, sizeof(g_line_cache_valid));
    memset(g_line_cache_vram_last, 0, sizeof(g_line_cache_vram_last));
    memset(g_line_cache_cgram_last, 0, sizeof(g_line_cache_cgram_last));
    memset(g_line_cache_oam_last, 0, sizeof(g_line_cache_oam_last));
    next = 1;
  }
  *serial = *last = next;
}

static inline void PpuLineCacheVram(uint32_t adr) {
  if (g_line_cache_tracking)
    g_line_cache_cur_vram[(adr >> (kLineCacheVramShift + 5)) & (kLineCacheVramWords - 1)] |=
        1u << ((adr >> kLineCacheVramShift) & 31);
}

static bool PpuLineCacheChanged(const uint32_t *dep, int words,
                                const uint32_t *last, uint32_t at) {
  for (int w = 0; w < words; w++) {
    uint32_t bits = dep[w];
    while (bits) {
      int bit = __builtin_ctz(bits);
      if (last[w * 32 + bit] > at) return true;
      bits &= bits - 1;
    }
  }
  return false;
}

/* Compare PpuLineCacheState excluding oamAdr (write-only pointer, no visual
 * effect) and m7matrix (only relevant in mode 7; games write dummy values
 * every frame in other modes).  Probe-validated: this unlocks ~96.6% cache
 * hit rate during ALttP gameplay with 0 false positives. */
static bool PpuLineCacheRegsMatch(const PpuLineCacheState *a, const PpuLineCacheState *b) {
  const uint8_t *pa = (const uint8_t *)a;
  const uint8_t *pb = (const uint8_t *)b;
  size_t oamAdr_off = offsetof(PpuLineCacheState, oamAdr);
  size_t oamAdr_end = oamAdr_off + sizeof(a->oamAdr);
  size_t m7_off = offsetof(PpuLineCacheState, m7matrix);
  size_t m7_end = m7_off + sizeof(a->m7matrix);
  bool skip_m7 = (a->mode != 7 && b->mode != 7);
  for (size_t i = 0; i < sizeof(PpuLineCacheState); i++) {
    if (i >= oamAdr_off && i < oamAdr_end) continue;
    if (skip_m7 && i >= m7_off && i < m7_end) continue;
    if (pa[i] != pb[i]) return false;
  }
  return true;
}

static bool PpuLineCacheCanReuse(Ppu *ppu, int line, bool eligible) {
  int y = line - 1;
  uint32_t bucket = (g_line_cache_frame - 1) / 300;
  if (bucket >= kLineCacheBuckets) bucket = kLineCacheBuckets - 1;
  g_line_cache_stats[bucket].total++;
  g_line_cache_stats[kLineCacheBuckets].total++;
  if (!eligible)
    return false;
  PpuLineCacheCapture(&g_line_cache_current, ppu);
  if (!g_line_cache_valid[y])
    return false;
  if (ppu->renderPitch == 0
#ifdef TARGET_GNW
      || g_ppu_line_cb != NULL
#endif
      ||
      !PpuLineCacheRegsMatch(&g_line_cache_current, &g_line_cache_state[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  if (PpuLineCacheChanged(g_line_cache_vram_dep[y], kLineCacheVramWords,
                          g_line_cache_vram_last,
                          g_line_cache_vram_at[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  if (PpuLineCacheChanged(g_line_cache_cgram_dep[y], 8, g_line_cache_cgram_last,
                          g_line_cache_cgram_at[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  uint32_t oam_union[4];
  for (int w = 0; w < 4; w++) oam_union[w] = g_line_cache_oam_dep[y][w] | ppu->objLineCand[y][w];
  if (PpuLineCacheChanged(oam_union, 4, g_line_cache_oam_last,
                          g_line_cache_oam_at[y])) {
    PpuLineCacheMiss(y);
    return false;
  }
  g_line_cache_stats[bucket].hits++;
  g_line_cache_stats[kLineCacheBuckets].hits++;
  return true;
}

static void PpuLineCacheCommit(Ppu *ppu, int line) {
  int y = line - 1;
  g_line_cache_state[y] = g_line_cache_current;
  memcpy(g_line_cache_vram_dep[y], g_line_cache_cur_vram, sizeof(g_line_cache_cur_vram));
  memcpy(g_line_cache_oam_dep[y], ppu->objLineCand[y], sizeof(g_line_cache_oam_dep[y]));
  memset(g_line_cache_cgram_dep[y], 0, sizeof(g_line_cache_cgram_dep[y]));
  uint32_t math_enabled = 0;
  for (int layer = 0; layer < 6; layer++) math_enabled |= ppu->mathEnabled[layer] << layer;
  bool uses_subscreen = ppu->preventMathMode != 3 && ppu->addSubscreen &&
      math_enabled && ppu->screenEnabled[1] != 0;
  for (int x = 0; x < kPpuXPixels; x++) {
    uint8_t index = ppu->bgBuffers[0].data[x] & 0xff;
    g_line_cache_cgram_dep[y][index >> 5] |= 1u << (index & 31);
    if (uses_subscreen) {
      index = ppu->bgBuffers[1].data[x] & 0xff;
      g_line_cache_cgram_dep[y][index >> 5] |= 1u << (index & 31);
    }
  }
  g_line_cache_vram_at[y] = g_line_cache_vram_serial;
  g_line_cache_cgram_at[y] = g_line_cache_cgram_serial;
  g_line_cache_oam_at[y] = g_line_cache_oam_serial;
  g_line_cache_valid[y] = 1;
}

void ppu_lineCacheReport(void) {
  static const char *const names[kLineCacheBuckets + 1] = {
    "0-299", "300-599", "600-899", "900-1199", "all"
  };
  for (int b = 0; b <= kLineCacheBuckets; b++) {
    const PpuLineCacheStats *s = &g_line_cache_stats[b];
    printf("[line-cache] bucket=%s total=%llu hits=%llu hit_x10000=%llu metadata=%u\n",
      names[b], (unsigned long long)s->total, (unsigned long long)s->hits,
      (unsigned long long)(s->total ? (uint64_t)s->hits * 10000 / s->total : 0),
      (unsigned)(sizeof(g_line_cache_state) + sizeof(g_line_cache_current) +
                 sizeof(g_line_cache_vram_dep) +
                 sizeof(g_line_cache_cgram_dep) + sizeof(g_line_cache_oam_dep) +
                 sizeof(g_line_cache_cur_vram) +
                 sizeof(g_line_cache_vram_last) + sizeof(g_line_cache_cgram_last) +
                 sizeof(g_line_cache_oam_last) + sizeof(g_line_cache_vram_at) +
                 sizeof(g_line_cache_cgram_at) + sizeof(g_line_cache_oam_at) +
                 sizeof(g_line_cache_valid) + sizeof(g_line_cache_cooldown) +
                 sizeof(g_line_cache_vram_serial) + sizeof(g_line_cache_cgram_serial) +
                 sizeof(g_line_cache_oam_serial) + sizeof(g_line_cache_frame) +
                 sizeof(g_line_cache_tracking) + sizeof(g_line_cache_stats)));
  }
}

void ppu_lineCacheInvalidate(void) {
  memset(g_line_cache_valid, 0, sizeof(g_line_cache_valid));
  memset(g_line_cache_cooldown, 0, sizeof(g_line_cache_cooldown));
}
#endif

#if defined(SNES_LINE_REUSE_PROBE) || defined(SNES_LINE_CACHE)
static inline void PpuTrackVramAdr(uint32_t adr) {
#ifdef SNES_LINE_REUSE_PROBE
  PpuLineProbeVram(adr);
#endif
#ifdef SNES_LINE_CACHE
  PpuLineCacheVram(adr);
#endif
}
static inline uint16_t PpuTrackVramPtr(Ppu *ppu, const uint16_t *ptr) {
  PpuTrackVramAdr((uint32_t)(ptr - ppu->vram) & 0x7fff);
  return *ptr;
}
#define PPU_PROBE_VRAM_ADR(adr) PpuTrackVramAdr((adr) & 0x7fff)
#define PPU_PROBE_VRAM_PTR(ppu, ptr) PpuTrackVramPtr((ppu), (ptr))
#else
#define PPU_PROBE_VRAM_ADR(adr) ((void)0)
#define PPU_PROBE_VRAM_PTR(ppu, ptr) (*(ptr))
#endif

void ppu_runLine(Ppu* ppu, int line) {
  if(line == 0) {
#ifdef SNES_LINE_REUSE_PROBE
    g_probe_frame++;
#endif
#ifdef SNES_LINE_CACHE
    g_line_cache_frame++;
#endif
    // pre-render line
    // TODO: this now happens halfway into the first line
    ppu->mosaicStartLine = 1;
    ppu->rangeOver = false;
    ppu->timeOver = false;
    ppu->evenFrame = !ppu->evenFrame;
  } else {  
    // Cache the brightness computation
    if (ppu->brightness != ppu->lastBrightnessMult) {
      uint8_t ppu_brightness = ppu->brightness;
      ppu->lastBrightnessMult = ppu_brightness;
      for (int i = 0; i < 32; i++)
        ppu->brightnessMultHalf[i * 2] = ppu->brightnessMultHalf[i * 2 + 1] = ppu->brightnessMult[i] =
        ((i << 3) | (i >> 2)) * ppu_brightness / 15;
      // Store 31 extra entries to remove the need for clamping to 31.
      memset(&ppu->brightnessMult[32], ppu->brightnessMult[31], 31);
#ifdef PPU_RGB565
      ppu->paletteDirty = true;
#endif
    }

    // evaluate sprites. The buffer only needs wiping if the previous line put
    // something in it — most lines of most frames have no sprites at all.
#ifdef SNES_LINE_REUSE_PROBE
    memset(g_probe_cur_vram_mask, 0, sizeof(g_probe_cur_vram_mask));
#endif
#ifdef SNES_LINE_CACHE
    bool cache_eligible = ppu->mode != 7 && PpuLineCacheBeginLine(line - 1);
    g_line_cache_tracking = cache_eligible;
    if (cache_eligible)
      memset(g_line_cache_cur_vram, 0, sizeof(g_line_cache_cur_vram));
#endif
    if (!ppu->objBufferClean)
      ClearBackdrop(&ppu->objBuffer);
    ppu->lineHasSprites = !ppu->forcedBlank && ppu_evaluateSprites(ppu, line - 1);
    ppu->objBufferClean = !ppu->lineHasSprites;

    if (g_ppu_skip_render)
      return;   /* frameskip: the flags above still matter, the pixels below do not */

    if (g_new_ppu) {
#ifdef SNES_LINE_CACHE
      bool reused = PpuLineCacheCanReuse(ppu, line, cache_eligible);
      if (!reused) {
#endif
#ifdef SNES_LINE_REUSE_PROBE
        PpuLineProbeBefore(ppu, line);
#endif
        PpuDrawWholeLine(ppu, line);
#ifdef SNES_LINE_REUSE_PROBE
        PpuLineProbeAfter(ppu, line);
#endif
#ifdef SNES_LINE_CACHE
        if (cache_eligible && !g_line_cache_cooldown[line - 1])
          PpuLineCacheCommit(ppu, line);
      }
      g_line_cache_tracking = false;
#endif
    } else {
      // actual line
      if (ppu->mode == 7) ppu_calculateMode7Starts(ppu, line);
      for (int x = 0; x < 256; x++) {
        ppu_handlePixel(ppu, x, line);
      }
    }
  }
}

typedef struct PpuWindows {
  int16 edges[6];
  uint8 nr;
  uint8 bits;
} PpuWindows;

static void PpuWindows_Clear(PpuWindows *win, Ppu *ppu, uint layer) {
  win->edges[0] = -(layer != 2 ? ppu->extraLeftCur : 0);
  win->edges[1] = 256 + (layer != 2 ? ppu->extraRightCur : 0);
  win->nr = 1;
  win->bits = 0;
}

static void PpuWindows_Calc(PpuWindows *win, Ppu *ppu, uint layer) {
  // Evaluate which spans to render based on the window settings.
  // There are at most 5 windows.
  // Algorithm from Snes9x
  uint32 winflags = GET_WINDOW_FLAGS(ppu, layer);
  uint nr = 1;
  int window_right = 256 + (layer != 2 ? ppu->extraRightCur : 0);
  win->edges[0] = - (layer != 2 ? ppu->extraLeftCur : 0);
  win->edges[1] = window_right;
  uint i, j;
  int t;
  bool w1_ena = (winflags & kWindow1Enabled) && ppu->window1left <= ppu->window1right;
  if (w1_ena) {
    if (ppu->window1left > win->edges[0]) {
      win->edges[nr] = ppu->window1left;
      win->edges[++nr] = window_right;
    }
    if (ppu->window1right + 1 < window_right) {
      win->edges[nr] = ppu->window1right + 1;
      win->edges[++nr] = window_right;
    }
  }
  bool w2_ena = (winflags & kWindow2Enabled) && ppu->window2left <= ppu->window2right;
  if (w2_ena) {
    for (i = 0; i <= nr && (t = ppu->window2left) != win->edges[i]; i++) {
      if (t < win->edges[i]) {
        for (j = nr++; j >= i; j--)
          win->edges[j + 1] = win->edges[j];
        win->edges[i] = t;
        break;
      }
    }
    for (; i <= nr && (t = ppu->window2right + 1) != win->edges[i]; i++) {
      if (t < win->edges[i]) {
        for (j = nr++; j >= i; j--)
          win->edges[j + 1] = win->edges[j];
        win->edges[i] = t;
        break;
      }
    }
  }
  win->nr = nr;
  // get a bitmap of how regions map to windows
  uint8 w1_bits = 0, w2_bits = 0;
  if (w1_ena) {
    for (i = 0; win->edges[i] != ppu->window1left; i++);
    for (j = i; win->edges[j] != ppu->window1right + 1; j++);
    w1_bits = ((1 << (j - i)) - 1) << i;
  }
  if ((winflags & (kWindow1Enabled | kWindow1Inversed)) == (kWindow1Enabled | kWindow1Inversed))
    w1_bits = ~w1_bits;
  if (w2_ena) {
    for (i = 0; win->edges[i] != ppu->window2left; i++);
    for (j = i; win->edges[j] != ppu->window2right + 1; j++);
    w2_bits = ((1 << (j - i)) - 1) << i;
  }
  if ((winflags & (kWindow2Enabled | kWindow2Inversed)) == (kWindow2Enabled | kWindow2Inversed))
    w2_bits = ~w2_bits;
  win->bits = w1_bits | w2_bits;
}

static inline uint32 PpuSpreadByteToNibbles(uint32 x) {
  /* Insert three zero bits between each source bit. Four spread bitplanes OR
   * directly into eight chunky 4bpp pixels, avoiding four extracts per pixel. */
  x = (x | x << 12) & 0x000f000f;
  x = (x | x << 6) & 0x03030303;
  return (x | x << 3) & 0x11111111;
}

static inline uint32 PpuDecode4bpp(uint32 bits) {
  return PpuSpreadByteToNibbles(bits & 0xff) |
         PpuSpreadByteToNibbles(bits >> 8 & 0xff) << 1 |
         PpuSpreadByteToNibbles(bits >> 16 & 0xff) << 2 |
         PpuSpreadByteToNibbles(bits >> 24) << 3;
}

static inline uint32 PpuDecode2bpp(uint32 bits) {
  return PpuSpreadByteToNibbles(bits & 0xff) |
         PpuSpreadByteToNibbles(bits >> 8) << 1;
}

// Draw a whole line of a 4bpp background layer into bgBuffers
static void PpuDrawBackground_4bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo) {
#define DO_PIXEL(i) do { \
  pixel = (bits >> i) & 1 | (bits >> (7 + i)) & 2 | (bits >> (14 + i)) & 4 | (bits >> (21 + i)) & 8; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_PIXEL_HFLIP(i) do { \
  pixel = (bits >> (7 - i)) & 1 | (bits >> (14 - i)) & 2 | (bits >> (21 - i)) & 4 | (bits >> (28 - i)) & 8; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 0xf; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 0xf; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define READ_BITS(ta, tile) (PPU_PROBE_VRAM_ADR((ta) + (tile) * 16), addr = &ppu->vram[((ta) + (tile) * 16) & 0x7fff], addr[0] | addr[8] << 16)
  enum { kPaletteShift = 6 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  y += bglayer->vScroll;
  int sc_offs = bglayer->tilemapAdr + (((y >> 3) & 0x1f) << 5);
  if ((y & 0x100) && bglayer->tilemapHigher)
    sc_offs += bglayer->tilemapWider ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (bglayer->tilemapWider ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = ppu->bgLayer[layer].tileAdr, pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);
  const uint16 *addr;
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    uint x = win.edges[windex] + bglayer->hScroll;
    uint w = win.edges[windex + 1] - win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + win.edges[windex] + kPpuExtraLeftRight;
    const uint16 *tp = tps[x >> 8 & 1] + ((x >> 3) & 0x1f);
    const uint16 *tp_last = tps[x >> 8 & 1] + 31;
    const uint16 *tp_next = tps[(x >> 8 & 1) ^ 1];
#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
    // Handle clipped pixels on left side
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      w -= curw;
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          bits >>= (x & 7), x += curw;
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --curw);
        } else {
          bits <<= (x & 7), x += curw;
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --curw);
        }
      } else {
        dstz += curw;
      }
    }
    // Handle full tiles in the middle
    while (w >= 8) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        uint32 chunky = PpuDecode4bpp(bits);
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          DO_CHUNKY_PIXEL(0); DO_CHUNKY_PIXEL(1); DO_CHUNKY_PIXEL(2); DO_CHUNKY_PIXEL(3);
          DO_CHUNKY_PIXEL(4); DO_CHUNKY_PIXEL(5); DO_CHUNKY_PIXEL(6); DO_CHUNKY_PIXEL(7);
        } else {
          DO_CHUNKY_PIXEL_HFLIP(0); DO_CHUNKY_PIXEL_HFLIP(1); DO_CHUNKY_PIXEL_HFLIP(2); DO_CHUNKY_PIXEL_HFLIP(3);
          DO_CHUNKY_PIXEL_HFLIP(4); DO_CHUNKY_PIXEL_HFLIP(5); DO_CHUNKY_PIXEL_HFLIP(6); DO_CHUNKY_PIXEL_HFLIP(7);
        }
      }
      dstz += 8, w -= 8;
    }
    // Handle remaining clipped part
    if (w) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --w);
        } else {
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --w);
        }
      }
    }
  }
#undef READ_BITS
#undef DO_CHUNKY_PIXEL_HFLIP
#undef DO_CHUNKY_PIXEL
#undef DO_PIXEL
#undef DO_PIXEL_HFLIP
}

// Draw a whole line of a 2bpp background layer into bgBuffers.
// top_mask: 0x2000 lets priority-set tiles take the unconditional-store fast
// path -- valid only when this layer's high priority tops every z drawn so
// far (mode 1 BG3). Pass 0 when it does not (mode 0), forcing the z test.
static void PpuDrawBackground_2bpp(Ppu *ppu, uint y, bool sub, uint layer, PpuZbufType zhi, PpuZbufType zlo, uint16 top_mask) {
#define DO_PIXEL(i) do { \
  pixel = (bits >> i) & 1 | (bits >> (7 + i)) & 2; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_PIXEL_HFLIP(i) do { \
  pixel = (bits >> (7 - i)) & 1 | (bits >> (14 - i)) & 2; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 3; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 3; \
  if (pixel && z > dstz[i]) dstz[i] = z + pixel; } while (0)
#define DO_TOP_CHUNKY_PIXEL(i) do { \
  pixel = (chunky >> (4 * i)) & 3; \
  if (pixel) dstz[i] = z + pixel; } while (0)
#define DO_TOP_CHUNKY_PIXEL_HFLIP(i) do { \
  pixel = (chunky >> (4 * (7 - i))) & 3; \
  if (pixel) dstz[i] = z + pixel; } while (0)
#define READ_BITS(ta, tile) (PPU_PROBE_VRAM_ADR((ta) + (tile) * 8), addr = &ppu->vram[(ta) + (tile) * 8 & 0x7fff], addr[0])
  enum { kPaletteShift = 8 };
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  BgLayer *bglayer = &ppu->bgLayer[layer];
  y += bglayer->vScroll;
  int sc_offs = bglayer->tilemapAdr + (((y >> 3) & 0x1f) << 5);
  if ((y & 0x100) && bglayer->tilemapHigher)
    sc_offs += bglayer->tilemapWider ? 0x800 : 0x400;
  const uint16 *tps[2] = {
    &ppu->vram[sc_offs & 0x7fff],
    &ppu->vram[sc_offs + (bglayer->tilemapWider ? 0x400 : 0) & 0x7fff]
  };
  int tileadr = ppu->bgLayer[layer].tileAdr, pixel;
  int tileadr1 = tileadr + 7 - (y & 0x7), tileadr0 = tileadr + (y & 0x7);

  const uint16 *addr;
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    uint x = win.edges[windex] + bglayer->hScroll;
    uint w = win.edges[windex + 1] - win.edges[windex];
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + win.edges[windex] + kPpuExtraLeftRight;
    const uint16 *tp = tps[x >> 8 & 1] + ((x >> 3) & 0x1f);
    const uint16 *tp_last = tps[x >> 8 & 1] + 31;
    const uint16 *tp_next = tps[(x >> 8 & 1) ^ 1];

#define NEXT_TP() if (tp != tp_last) tp += 1; else tp = tp_next, tp_next = tp_last - 31, tp_last = tp + 31;
    // Handle clipped pixels on left side
    if (x & 7) {
      int curw = IntMin(8 - (x & 7), w);
      w -= curw;
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          bits >>= (x & 7), x += curw;
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --curw);
        } else {
          bits <<= (x & 7), x += curw;
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --curw);
        }
      } else {
        dstz += curw;
      }
    }
    // Handle full tiles in the middle
    while (w >= 8) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      NEXT_TP();
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        uint32 chunky = PpuDecode2bpp(bits);
        z += ((tile & 0x1c00) >> kPaletteShift);
        /* In mode 1 this renderer is BG3, whose high priority (0xf2) is above
         * every BG1/BG2/OBJ priority, so the z test is always true and the
         * TOP store can skip it (top_mask = 0x2000). Mode 0 layers have
         * sprites above them at every priority, so they pass top_mask = 0. */
        if (tile & 0x4000) {
          if (tile & top_mask) {
            DO_TOP_CHUNKY_PIXEL(0); DO_TOP_CHUNKY_PIXEL(1); DO_TOP_CHUNKY_PIXEL(2); DO_TOP_CHUNKY_PIXEL(3);
            DO_TOP_CHUNKY_PIXEL(4); DO_TOP_CHUNKY_PIXEL(5); DO_TOP_CHUNKY_PIXEL(6); DO_TOP_CHUNKY_PIXEL(7);
          } else {
            DO_CHUNKY_PIXEL(0); DO_CHUNKY_PIXEL(1); DO_CHUNKY_PIXEL(2); DO_CHUNKY_PIXEL(3);
            DO_CHUNKY_PIXEL(4); DO_CHUNKY_PIXEL(5); DO_CHUNKY_PIXEL(6); DO_CHUNKY_PIXEL(7);
          }
        } else {
          if (tile & top_mask) {
            DO_TOP_CHUNKY_PIXEL_HFLIP(0); DO_TOP_CHUNKY_PIXEL_HFLIP(1); DO_TOP_CHUNKY_PIXEL_HFLIP(2); DO_TOP_CHUNKY_PIXEL_HFLIP(3);
            DO_TOP_CHUNKY_PIXEL_HFLIP(4); DO_TOP_CHUNKY_PIXEL_HFLIP(5); DO_TOP_CHUNKY_PIXEL_HFLIP(6); DO_TOP_CHUNKY_PIXEL_HFLIP(7);
          } else {
            DO_CHUNKY_PIXEL_HFLIP(0); DO_CHUNKY_PIXEL_HFLIP(1); DO_CHUNKY_PIXEL_HFLIP(2); DO_CHUNKY_PIXEL_HFLIP(3);
            DO_CHUNKY_PIXEL_HFLIP(4); DO_CHUNKY_PIXEL_HFLIP(5); DO_CHUNKY_PIXEL_HFLIP(6); DO_CHUNKY_PIXEL_HFLIP(7);
          }
        }
      }
      dstz += 8, w -= 8;
    }
    // Handle remaining clipped part
    if (w) {
      uint32 tile = PPU_PROBE_VRAM_PTR(ppu, tp);
      int ta = (tile & 0x8000) ? tileadr1 : tileadr0;
      PpuZbufType z = (tile & 0x2000) ? zhi : zlo;
      uint32 bits = READ_BITS(ta, tile & 0x3ff);
      if (bits) {
        z += ((tile & 0x1c00) >> kPaletteShift);
        if (tile & 0x4000) {
          do DO_PIXEL(0); while (bits >>= 1, dstz++, --w);
        } else {
          do DO_PIXEL_HFLIP(0); while (bits <<= 1, dstz++, --w);
        }
      }
    }
  }
#undef NEXT_TP
#undef READ_BITS
#undef DO_TOP_CHUNKY_PIXEL_HFLIP
#undef DO_TOP_CHUNKY_PIXEL
#undef DO_CHUNKY_PIXEL_HFLIP
#undef DO_CHUNKY_PIXEL
#undef DO_PIXEL
#undef DO_PIXEL_HFLIP
}

// Assumes it's drawn on an empty backdrop
static void PpuDrawBackground_mode7(Ppu *ppu, uint y, bool sub, PpuZbufType z) {
  int layer = 0;
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);

  // expand 13-bit values to signed values
  int hScroll = ((int16_t)(ppu->m7matrix[6] << 3)) >> 3;
  int vScroll = ((int16_t)(ppu->m7matrix[7] << 3)) >> 3;
  int xCenter = ((int16_t)(ppu->m7matrix[4] << 3)) >> 3;
  int yCenter = ((int16_t)(ppu->m7matrix[5] << 3)) >> 3;
  int clippedH = hScroll - xCenter;
  int clippedV = vScroll - yCenter;
  clippedH = (clippedH & 0x2000) ? (clippedH | ~1023) : (clippedH & 1023);
  clippedV = (clippedV & 0x2000) ? (clippedV | ~1023) : (clippedV & 1023);
  bool mosaic_enabled = IS_MOSAIC_ENABLED(ppu, 0);
  if (mosaic_enabled)
    y = ppu->mosaicModulo[y];
  uint32 ry = ppu->m7yFlip ? 255 - y : y;
  uint32 m7startX = (ppu->m7matrix[0] * clippedH & ~63) + (ppu->m7matrix[1] * ry & ~63) +
    (ppu->m7matrix[1] * clippedV & ~63) + (xCenter << 8);
  uint32 m7startY = (ppu->m7matrix[2] * clippedH & ~63) + (ppu->m7matrix[3] * ry & ~63) +
    (ppu->m7matrix[3] * clippedV & ~63) + (yCenter << 8);
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int x = win.edges[windex], x2 = win.edges[windex + 1], tile;
    PpuZbufType *dstz = ppu->bgBuffers[sub].data + x + kPpuExtraLeftRight;
    PpuZbufType *dstz_end = ppu->bgBuffers[sub].data + x2 + kPpuExtraLeftRight;
    uint32 rx = ppu->m7xFlip ? 255 - x : x;
    uint32 xpos = m7startX + ppu->m7matrix[0] * rx;
    uint32 ypos = m7startY + ppu->m7matrix[2] * rx;
    uint32 dx = ppu->m7xFlip ? -ppu->m7matrix[0] : ppu->m7matrix[0];
    uint32 dy = ppu->m7xFlip ? -ppu->m7matrix[2] : ppu->m7matrix[2];
    uint32 outside_value = ppu->m7largeField ? 0x3ffff : 0xffffffff;
    bool char_fill = ppu->m7charFill;
    if (mosaic_enabled) {
      int w = ppu->mosaicSize - (x - ppu->mosaicModulo[x]);
      do {
        w = IntMin(w, dstz_end - dstz);
        if ((uint32)(xpos | ypos) > outside_value) {
          if (!char_fill)
            continue;
          tile = 0;
        } else {
          uint32_t map_adr = (ypos >> 11 & 0x7f) * 128 + (xpos >> 11 & 0x7f);
          tile = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[map_adr]) & 0xff;
        }
        uint8 pixel = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[tile * 64 + (ypos >> 8 & 7) * 8 + (xpos >> 8 & 7)]) >> 8;
        if (pixel) {
          int i = 0;
          do dstz[i] = pixel + z; while (++i != w);
        }
      } while (xpos += dx * w, ypos += dy * w, dstz += w, w = ppu->mosaicSize, dstz_end - dstz != 0);
    } else {
      do {
        if ((uint32)(xpos | ypos) > outside_value) {
          if (!char_fill)
            continue;
          tile = 0;
        } else {
          uint32_t map_adr = (ypos >> 11 & 0x7f) * 128 + (xpos >> 11 & 0x7f);
          tile = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[map_adr]) & 0xff;
        }
        uint8 pixel = PPU_PROBE_VRAM_PTR(ppu, &ppu->vram[tile * 64 + (ypos >> 8 & 7) * 8 + (xpos >> 8 & 7)]) >> 8;
        if (pixel)
          dstz[0] = pixel + z;
      } while (xpos += dx, ypos += dy, ++dstz != dstz_end);
    }
  }
}


static void PpuDrawSprites(Ppu *ppu, uint y, uint sub, bool clear_backdrop) {
  int layer = 4;
  if (!IS_SCREEN_ENABLED(ppu, sub, layer))
    return;  // layer is completely hidden
  PpuWindows win;
  IS_SCREEN_WINDOWED(ppu, sub, layer) ? PpuWindows_Calc(&win, ppu, layer) : PpuWindows_Clear(&win, ppu, layer);
  for (size_t windex = 0; windex < win.nr; windex++) {
    if (win.bits & (1 << windex))
      continue;  // layer is disabled for this window part
    int left = win.edges[windex];
    int width = win.edges[windex + 1] - left;
    PpuZbufType *src = ppu->objBuffer.data + left + kPpuExtraLeftRight;
    PpuZbufType *dst = ppu->bgBuffers[sub].data + left + kPpuExtraLeftRight;
    if (clear_backdrop) {
      memcpy(dst, src, width * sizeof(uint16));
    } else {
      do {
        if (src[0] > dst[0])
          dst[0] = src[0];
      } while (src++, dst++, --width);
    }
  }
}

static void PpuDrawBackgrounds(Ppu *ppu, int y, bool sub) {
  // Top 4 bits contain the prio level, and bottom 4 bits the layer type.
  // SPRITE_PRIO_TO_PRIO can be used to convert from obj prio to this prio.
  //  15: BG3 tiles with priority 1 if bit 3 of $2105 is set
  //  14: Sprites with priority 3 (4 * sprite_prio + 2)
  //  12: BG1 tiles with priority 1
  //  11: BG2 tiles with priority 1
  //  10: Sprites with priority 2 (4 * sprite_prio + 2)
  //  8: BG1 tiles with priority 0
  //  7: BG2 tiles with priority 0
  //  6: Sprites with priority 1 (4 * sprite_prio + 2)
  //  3: BG3 tiles with priority 1 if bit 3 of $2105 is clear
  //  2: Sprites with priority 0 (4 * sprite_prio + 2)
  //  1: BG3 tiles with priority 0
  //  0: backdrop

  if (ppu->mode == 1) {
    if (ppu->lineHasSprites)
      PpuDrawSprites(ppu, y, sub, true);

#ifdef GNW_SNES_CORE
    /* General-purpose core: mosaic is a screen-transition effect half the
     * commercial library uses (fades in Zelda, F-Zero, menu wipes...). This
     * renderer has no mosaic path -- draw the background UN-mosaiced instead
     * of dying: the transition looks plain, the game keeps running. The
     * asserts stay for the sm/zelda3 dev builds below, where hitting one
     * means the port needs a real mosaic implementation for that game. */
    PpuDrawBackground_4bpp(ppu, y, sub, 0, 0xc000, 0x8000);
    PpuDrawBackground_4bpp(ppu, y, sub, 1, 0xb100, 0x7100);
    PpuDrawBackground_2bpp(ppu, y, sub, 2, 0xf200, 0x1200, 0x2000);
#else
    if (IS_MOSAIC_ENABLED(ppu, 0))
      assert(0);
    else
      PpuDrawBackground_4bpp(ppu, y, sub, 0, 0xc000, 0x8000);

    if (IS_MOSAIC_ENABLED(ppu, 1))
      assert(0);
    else
      PpuDrawBackground_4bpp(ppu, y, sub, 1, 0xb100, 0x7100);

    if (IS_MOSAIC_ENABLED(ppu, 2))
      assert(0);
    else
      PpuDrawBackground_2bpp(ppu, y, sub, 2, 0xf200, 0x1200, 0x2000);
#endif
  } else if (ppu->mode == 0) {
    /* Mode 0: four 2bpp layers, each with its own 32-colour CGRAM window
     * (BG2 +32, BG3 +64, BG4 +96 -- folded into the z parameters, whose low
     * byte is the CGRAM index). Priority ranks interleave with the sprite
     * ranks (4*prio+2 = 2/6/10/14) in the hardware order
     * S3 BG1p1 BG2p1 S2 BG1p0 BG2p0 S1 BG3p1 BG4p1 S0 BG3p0 BG4p0.
     * Sprites are never below any layer's fast path here, so top_mask = 0.
     * (Mario Kart's whole menu flow -- driver select included -- is mode 0;
     * this used to fall through to the mode-7 renderer and drew garbage.) */
    if (ppu->lineHasSprites)
      PpuDrawSprites(ppu, y, sub, true);
    PpuDrawBackground_2bpp(ppu, y, sub, 0, 0xd000,      0x9000,      0);
    PpuDrawBackground_2bpp(ppu, y, sub, 1, 0xc100 + 32, 0x8100 + 32, 0);
    PpuDrawBackground_2bpp(ppu, y, sub, 2, 0x5200 + 64, 0x1200 + 64, 0);
    PpuDrawBackground_2bpp(ppu, y, sub, 3, 0x4300 + 96, 0x0300 + 96, 0);
  } else {
    // mode 7
    PpuDrawBackground_mode7(ppu, y, sub, 0x5000);
    if (ppu->lineHasSprites)
      PpuDrawSprites(ppu, y, sub, false);
  }
}

#ifdef PPU_RGB565
static void PpuRebuildPalette(Ppu *ppu) {
  for (int i = 0; i < 256; i++) {
    uint32 color = ppu->cgram[i];
    ppu->palette565[i] = (uint16_t)(
        (ppu->brightnessMult[color & 0x1f] >> 3) << 11 |
        (ppu->brightnessMult[(color >> 5) & 0x1f] >> 2) << 5 |
        (ppu->brightnessMult[(color >> 10) & 0x1f] >> 3));
  }
  ppu->paletteDirty = false;
}

static uint32_t PpuMathFixedKey(Ppu *ppu) {
  uint32_t key = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
  key |= (uint32_t)ppu->subtractColor << 15;
  key |= (uint32_t)ppu->halfColor << 16;
  key |= (uint32_t)ppu->addSubscreen << 17;
  for (int layer = 0; layer < 6; layer++)
    key |= (uint32_t)ppu->mathEnabled[layer] << (18 + layer);
  return key;
}

static void PpuRebuildMathFixed(Ppu *ppu, uint32_t key) {
  uint32_t fixed = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
  uint32_t r2 = fixed & 0x1f, g2 = fixed >> 5 & 0x1f, b2 = fixed >> 10 & 0x1f;
  /* With addSubscreen enabled, a transparent subscreen pixel falls back to the
   * fixed color but is deliberately NOT halved (SNES rule, matching the old
   * loop). Otherwise fixed-color half math uses brightnessMultHalf. */
  uint8_t *math_map = ppu->halfColor && !ppu->addSubscreen ?
      ppu->brightnessMultHalf : ppu->brightnessMult;
  for (int clip = 0; clip < 2; clip++) {
    uint32_t mask = clip ? 0x1f : 0;
    for (int layer = 0; layer < 6; layer++) {
      bool do_math = ppu->mathEnabled[layer];
      for (int index = 0; index < 256; index++) {
        uint32_t color = ppu->cgram[index];
        uint32_t r = color & mask, g = color >> 5 & mask, b = color >> 10 & mask;
        uint8_t *color_map = ppu->brightnessMult;
        if (do_math) {
          color_map = math_map;
          if (ppu->subtractColor) {
            r = r >= r2 ? r - r2 : 0;
            g = g >= g2 ? g - g2 : 0;
            b = b >= b2 ? b - b2 : 0;
          } else {
            r += r2, g += g2, b += b2;
          }
        }
        ppu->mathFixed565[clip][layer][index] =
            (color_map[b] >> 3) | (color_map[g] >> 2) << 5 | (color_map[r] >> 3) << 11;
      }
    }
  }
  ppu->mathFixedKey = key;
}

#endif

static NOINLINE void PpuDrawWholeLine(Ppu *ppu, uint y) {
#ifdef PPU_RGB565
  bool palette_was_dirty = ppu->paletteDirty;
  if (palette_was_dirty)
    PpuRebuildPalette(ppu);   /* cgram or brightness moved since the last line */
  uint32_t math_fixed_key = PpuMathFixedKey(ppu);
  if (palette_was_dirty || math_fixed_key != ppu->mathFixedKey)
    PpuRebuildMathFixed(ppu, math_fixed_key);
#endif
  if (ppu->forcedBlank) {
    uint8 *dst = &ppu->renderBuffer[(y - 1) * ppu->renderPitch];
#ifdef PPU_RGB565
    size_t n = sizeof(uint16_t) * (256 + ppu->extraLeftRight * 2);
#else
    size_t n = sizeof(uint32) * (256 + ppu->extraLeftRight * 2);
#endif
    memset(dst, 0, n);
#ifdef TARGET_GNW
    if (g_ppu_line_cb)
      g_ppu_line_cb(y, (const uint16_t *)dst);
#endif
    return;
  }

  // Default background is backdrop
  ClearBackdrop(&ppu->bgBuffers[0]);

  // Render main screen
  PpuDrawBackgrounds(ppu, y, false);

  // The 6:th bit is automatically zero, math is never applied to the first half of the sprites.
  uint32 math_enabled = 0;
  for(int i = 0; i < 6; i++)
    math_enabled |= ppu->mathEnabled[i] << i;

  // Render also the subscreen?
  bool rendered_subscreen = false;
  if (ppu->preventMathMode != 3 && ppu->addSubscreen && math_enabled) {
    ClearBackdrop(&ppu->bgBuffers[1]);
    if (ppu->screenEnabled[1] != 0) {
      PpuDrawBackgrounds(ppu, y, true);
      rendered_subscreen = true;
    }
  }

  // Color window affects the drawing mode in each region
  PpuWindows cwin;
  PpuWindows_Calc(&cwin, ppu, 5);
  static const uint8 kCwBitsMod[8] = {
    0x00, 0xff, 0xff, 0x00,
    0xff, 0x00, 0xff, 0x00,
  };
  uint32 cw_clip_math = ((cwin.bits & kCwBitsMod[ppu->clipMode]) ^ kCwBitsMod[ppu->clipMode + 4]) |
    ((cwin.bits & kCwBitsMod[ppu->preventMathMode]) ^ kCwBitsMod[ppu->preventMathMode + 4]) << 8;

#ifdef PPU_RGB565
  uint16_t *dst = (uint16_t*)&ppu->renderBuffer[(y - 1) * ppu->renderPitch], *dst_org = dst;
#else
  uint32 *dst = (uint32*)&ppu->renderBuffer[(y - 1) * ppu->renderPitch], *dst_org = dst;
#endif

  dst += (ppu->extraLeftRight - ppu->extraLeftCur);

  uint32 windex = 0;
  do {
    uint32 left = cwin.edges[windex] + kPpuExtraLeftRight, right = cwin.edges[windex + 1] + kPpuExtraLeftRight;
    // If clip is set, then zero out the rgb values from the main screen.
    uint32 clip_color_mask = (cw_clip_math & 1) ? 0x1f : 0;
    uint32 math_enabled_cur = (cw_clip_math & 0x100) ? math_enabled : 0;
    uint32 fixed_color = ppu->fixedColorR | ppu->fixedColorG << 5 | ppu->fixedColorB << 10;
    if (math_enabled_cur == 0 || fixed_color == 0 && !ppu->halfColor && !rendered_subscreen) {
      // Math is disabled (or has no effect), so can avoid the per-pixel maths check
      uint32 i = left;
#ifdef PPU_RGB565
      if (clip_color_mask == 0x1f) {
        const uint16_t *pal = ppu->palette565;
        const PpuZbufType *src = ppu->bgBuffers[0].data;
        /* Two pixels per iteration: one 32-bit load of two z-entries, one 32-bit
         * store of two RGB565 pixels (little-endian pairing, like the 64-bit fill
         * in ClearBackdrop). src and dst advance in lockstep, so when they are
         * co-aligned one odd head pixel word-aligns both; when they are not
         * (odd render pitch), the plain tail loop does the whole span. */
        if ((((uintptr_t)dst ^ (uintptr_t)&src[i]) & 3) == 0) {
          if ((uintptr_t)dst & 3)
            dst[0] = pal[src[i] & 0xff], dst++, i++;
          for (; i + 1 < right; i += 2, dst += 2) {
            uint32 zz = *(const uint32 *)&src[i];
            *(uint32 *)dst = pal[zz & 0xff] | (uint32)pal[(zz >> 16) & 0xff] << 16;
          }
        }
        for (; i < right; i++, dst++)
          dst[0] = pal[src[i] & 0xff];
      } else {
        /* clip: every component masks to index 0, and brightnessMult[0] is 0 */
        do {
          dst[0] = 0;
        } while (dst++, ++i < right);
      }
#else
      do {
        uint32 color = ppu->cgram[ppu->bgBuffers[0].data[i] & 0xff];
        dst[0] = ppu->brightnessMult[color & clip_color_mask] << 16 |
          ppu->brightnessMult[(color >> 5) & clip_color_mask] << 8 |
          ppu->brightnessMult[(color >> 10) & clip_color_mask];
      } while (dst++, ++i < right);
#endif
    } else {
#if defined(PPU_RGB565) && defined(SNES_PPU_DIRECT_MATH)
      /* No subscreen means the result is solely a function of the main z/color
       * word and clip state.  Its low 12 bits are already laid out exactly as
       * mathFixed565[layer][index]. One lookup replaces math-enable branches
       * and the per-pixel fixed/subscreen test. Layer 6 (OBJ palettes exempt
       * from color math) falls back to the already-built plain palette. */
      if (!ppu->addSubscreen) {
        const uint16_t *direct = &ppu->mathFixed565[clip_color_mask != 0][0][0];
        const PpuZbufType *src = ppu->bgBuffers[0].data;
        uint32 i = left;
        do {
          uint32 main_z = src[i];
          uint32 layer = main_z >> 8 & 0xf;
          dst[0] = layer < 6 ? direct[(layer << 8) | (main_z & 0xff)] :
              (clip_color_mask ? ppu->palette565[main_z & 0xff] : 0);
        } while (dst++, ++i < right);
        continue;
      }
#endif
      uint8 *half_color_map = ppu->halfColor ? ppu->brightnessMultHalf : ppu->brightnessMult;
      /* The z word already stores [layer:4][CGRAM index:8] in its low 12 bits,
       * exactly matching the last two dimensions of mathFixed565. */
      const uint16_t *math_fixed = &ppu->mathFixed565[clip_color_mask != 0][0][0];
      // Store this in locals
      math_enabled_cur |= ppu->addSubscreen << 8 | ppu->subtractColor << 9;
      // Need to check for each pixel whether to use math or not based on the main screen layer.
      uint32 i = left;
      do {
        PpuZbufType main_z = ppu->bgBuffers[0].data[i];
        uint8 main_layer = (main_z >> 8) & 0xf;
        /* Fixed-color, transparent-subscreen AND math-disabled-layer pixels are
         * all a pure function of clip state, main layer and CGRAM index. When
         * this layer's mathEnabled bit is off, PpuRebuildMathFixed() built its
         * table entry with do_math=false — the exact same brightnessMult-only
         * formula the manual path below falls through to when the per-pixel
         * `math_enabled_cur & (1 << main_layer)` test fails. So a bypassing
         * pixel needs neither the real subscreen value nor the manual
         * extract/blend/repack below; it needs the same one lookup the
         * fixed-color case already uses. One lookup replaces component
         * extraction, layer test, add/subtract, clamp and RGB565 packing. */
        if (main_layer < 6 &&
            (!(math_enabled_cur & (1 << main_layer)) ||
             !ppu->addSubscreen || (ppu->bgBuffers[1].data[i] & 0xff) == 0)) {
          dst[0] = math_fixed[main_z & 0xfff];
          continue;
        }
        uint32 color = ppu->cgram[main_z & 0xff], color2;
        uint32 r = color & clip_color_mask;
        uint32 g = (color >> 5) & clip_color_mask;
        uint32 b = (color >> 10) & clip_color_mask;
        uint8 *color_map = ppu->brightnessMult;
        if (math_enabled_cur & (1 << main_layer)) {
          if (math_enabled_cur & 0x100) {  // addSubscreen ?
            if ((ppu->bgBuffers[1].data[i] & 0xff) != 0)
              color2 = ppu->cgram[ppu->bgBuffers[1].data[i] & 0xff], color_map = half_color_map;
            else  // Don't halve if ppu->addSubscreen && backdrop
              color2 = fixed_color;
          } else {
            color2 = fixed_color, color_map = half_color_map;
          }
          uint32 r2 = (color2 & 0x1f), g2 = ((color2 >> 5) & 0x1f), b2 = ((color2 >> 10) & 0x1f);
          if (math_enabled_cur & 0x200) {  // subtractColor?
            r = (r >= r2) ? r - r2 : 0;
            g = (g >= g2) ? g - g2 : 0;
            b = (b >= b2) ? b - b2 : 0;
          } else {
            r += r2;
            g += g2;
            b += b2;
          }
        }
#ifdef PPU_RGB565
        dst[0] = (color_map[b] >> 3) | (color_map[g] >> 2) << 5 | (color_map[r] >> 3) << 11;
#else
        dst[0] = color_map[b] | color_map[g] << 8 | color_map[r] << 16;
#endif
      } while (dst++, ++i < right);
    }
  } while (cw_clip_math >>= 1, ++windex < cwin.nr);

#ifdef TARGET_GNW
  if (g_ppu_line_cb)
    g_ppu_line_cb(y, dst_org);
#endif
}


static void ppu_handlePixel(Ppu* ppu, int x, int y) {
  int r = 0, r2 = 0;
  int g = 0, g2 = 0;
  int b = 0, b2 = 0;
  if(!ppu->forcedBlank) {
    int mainLayer = ppu_getPixel(ppu, x, y, false, &r, &g, &b);
    bool colorWindowState = ppu_getWindowState(ppu, 5, x);
    if(
      ppu->clipMode == 3 ||
      (ppu->clipMode == 2 && colorWindowState) ||
      (ppu->clipMode == 1 && !colorWindowState)
    ) {
      r = 0;
      g = 0;
      b = 0;
    }
    int secondLayer = 5; // backdrop
    bool mathEnabled = mainLayer < 6 && ppu->mathEnabled[mainLayer] && !(
      ppu->preventMathMode == 3 ||
      (ppu->preventMathMode == 2 && colorWindowState) ||
      (ppu->preventMathMode == 1 && !colorWindowState)
    );
    if((mathEnabled && ppu->addSubscreen) || ppu->pseudoHires || ppu->mode == 5 || ppu->mode == 6) {
      secondLayer = ppu_getPixel(ppu, x, y, true, &r2, &g2, &b2);
    }
    // TODO: subscreen pixels can be clipped to black as well
    // TODO: math for subscreen pixels (add/sub sub to main)
    if(mathEnabled) {
      if(ppu->subtractColor) {
        r -= (ppu->addSubscreen && secondLayer != 5) ? r2 : ppu->fixedColorR;
        g -= (ppu->addSubscreen && secondLayer != 5) ? g2 : ppu->fixedColorG;
        b -= (ppu->addSubscreen && secondLayer != 5) ? b2 : ppu->fixedColorB;
      } else {
        r += (ppu->addSubscreen && secondLayer != 5) ? r2 : ppu->fixedColorR;
        g += (ppu->addSubscreen && secondLayer != 5) ? g2 : ppu->fixedColorG;
        b += (ppu->addSubscreen && secondLayer != 5) ? b2 : ppu->fixedColorB;
      }
      if(ppu->halfColor && (secondLayer != 5 || !ppu->addSubscreen)) {
        r >>= 1;
        g >>= 1;
        b >>= 1;
      }
      if(r > 31) r = 31;
      if(g > 31) g = 31;
      if(b > 31) b = 31;
      if(r < 0) r = 0;
      if(g < 0) g = 0;
      if(b < 0) b = 0;
    }
    if(!(ppu->pseudoHires || ppu->mode == 5 || ppu->mode == 6)) {
      r2 = r; g2 = g; b2 = b;
    }
  }
  int row = y - 1;
#ifdef PPU_RGB565
  uint8 *pixelBuffer = (uint8*) &ppu->renderBuffer[row * ppu->renderPitch + (x + ppu->extraLeftRight) * 2];
  uint32 r8 = ((r << 3) | (r >> 2)) * ppu->brightness / 15;
  uint32 g8 = ((g << 3) | (g >> 2)) * ppu->brightness / 15;
  uint32 b8 = ((b << 3) | (b >> 2)) * ppu->brightness / 15;
  uint16_t px = (uint16_t)(((r8 >> 3) << 11) | ((g8 >> 2) << 5) | (b8 >> 3));
  pixelBuffer[0] = (uint8)px;
  pixelBuffer[1] = (uint8)(px >> 8);
#else
  uint8 *pixelBuffer = (uint8*) &ppu->renderBuffer[row * ppu->renderPitch + (x + ppu->extraLeftRight) * 4];
  pixelBuffer[0] = ((b << 3) | (b >> 2)) * ppu->brightness / 15;
  pixelBuffer[1] = ((g << 3) | (g >> 2)) * ppu->brightness / 15;
  pixelBuffer[2] = ((r << 3) | (r >> 2)) * ppu->brightness / 15;
  pixelBuffer[3] = 0;
#endif
}

static int ppu_getPixel(Ppu* ppu, int x, int y, bool sub, int* r, int* g, int* b) {
  // figure out which color is on this location on main- or subscreen, sets it in r, g, b
  // returns which layer it is: 0-3 for bg layer, 4 or 6 for sprites (depending on palette), 5 for backdrop
  int actMode = ppu->mode == 1 && ppu->bg3priority ? 8 : ppu->mode;
  actMode = ppu->mode == 7 && ppu->m7extBg ? 9 : actMode;
  int layer = 5;
  int pixel = 0;
  for(int i = 0; i < layerCountPerMode[actMode]; i++) {
    int curLayer = layersPerMode[actMode][i];
    int curPriority = prioritysPerMode[actMode][i];
    bool layerActive = false;
    if(!sub) {
      layerActive = ppu->layer[curLayer].mainScreenEnabled && (
        !ppu->layer[curLayer].mainScreenWindowed || !ppu_getWindowState(ppu, curLayer, x)
      );
    } else {
      layerActive = ppu->layer[curLayer].subScreenEnabled && (
        !ppu->layer[curLayer].subScreenWindowed || !ppu_getWindowState(ppu, curLayer, x)
      );
    }
    if(layerActive) {
      if(curLayer < 4) {
        // bg layer
        int lx = x;
        int ly = y;
        if(ppu->bgLayer[curLayer].mosaicEnabled && ppu->mosaicSize > 1) {
          lx -= lx % ppu->mosaicSize;
          ly -= (ly - ppu->mosaicStartLine) % ppu->mosaicSize;
        }
        if(ppu->mode == 7) {
          pixel = ppu_getPixelForMode7(ppu, lx, curLayer, curPriority);
        } else {
          lx += ppu->bgLayer[curLayer].hScroll;
          if(ppu->mode == 5 || ppu->mode == 6) {
            lx *= 2;
            lx += (sub || ppu->bgLayer[curLayer].mosaicEnabled) ? 0 : 1;
            if(ppu->interlace) {
              ly *= 2;
              ly += (ppu->evenFrame || ppu->bgLayer[curLayer].mosaicEnabled) ? 0 : 1;
            }
          }
          ly += ppu->bgLayer[curLayer].vScroll;
          if(ppu->mode == 2 || ppu->mode == 4 || ppu->mode == 6) {
            ppu_handleOPT(ppu, curLayer, &lx, &ly);
          }
          pixel = ppu_getPixelForBgLayer(
            ppu, lx & 0x3ff, ly & 0x3ff,
            curLayer, curPriority
          );
        }
      } else {
        // get a pixel from the sprite buffer
        pixel = 0;
        if ((ppu->objBuffer.data[x + kPpuExtraLeftRight] >> 12) == SPRITE_PRIO_TO_PRIO_HI(curPriority))
          pixel = ppu->objBuffer.data[x + kPpuExtraLeftRight] & 0xff;
      }
    }
    if(pixel > 0) {
      layer = curLayer;
      break;
    }
  }
  if(ppu->directColor && layer < 4 && bitDepthsPerMode[actMode][layer] == 8) {
    *r = ((pixel & 0x7) << 2) | ((pixel & 0x100) >> 7);
    *g = ((pixel & 0x38) >> 1) | ((pixel & 0x200) >> 8);
    *b = ((pixel & 0xc0) >> 3) | ((pixel & 0x400) >> 8);
  } else {
    uint16_t color = ppu->cgram[pixel & 0xff];
    *r = color & 0x1f;
    *g = (color >> 5) & 0x1f;
    *b = (color >> 10) & 0x1f;
  }
  if(layer == 4 && pixel < 0xc0) layer = 6; // sprites with palette color < 0xc0
  return layer;
}

static void ppu_handleOPT(Ppu* ppu, int layer, int* lx, int* ly) {
  int x = *lx;
  int y = *ly;
  int column = 0;
  if(ppu->mode == 6) {
    column = ((x - (x & 0xf)) - ((ppu->bgLayer[layer].hScroll * 2) & 0xfff0)) >> 4;
  } else {
    column = ((x - (x & 0x7)) - (ppu->bgLayer[layer].hScroll & 0xfff8)) >> 3;
  }
  if(column > 0) {
    // fetch offset values from layer 3 tilemap
    int valid = layer == 0 ? 0x2000 : 0x4000;
    uint16_t hOffset = ppu_getOffsetValue(ppu, column - 1, 0);
    uint16_t vOffset = 0;
    if(ppu->mode == 4) {
      if(hOffset & 0x8000) {
        vOffset = hOffset;
        hOffset = 0;
      }
    } else {
      vOffset = ppu_getOffsetValue(ppu, column - 1, 1);
    }
    if(ppu->mode == 6) {
      // TODO: not sure if correct
      if(hOffset & valid) *lx = (((hOffset & 0x3f8) + (column * 8)) * 2) | (x & 0xf);
    } else {
      if(hOffset & valid) *lx = ((hOffset & 0x3f8) + (column * 8)) | (x & 0x7);
    }
    // TODO: not sure if correct for interlace
    if(vOffset & valid) *ly = (vOffset & 0x3ff) + (y - ppu->bgLayer[layer].vScroll);
  }
}

static uint16_t ppu_getOffsetValue(Ppu* ppu, int col, int row) {
  int x = col * 8 + ppu->bgLayer[2].hScroll;
  int y = row * 8 + ppu->bgLayer[2].vScroll;
  int tileBits = ppu->bgLayer[2].bigTiles ? 4 : 3;
  int tileHighBit = ppu->bgLayer[2].bigTiles ? 0x200 : 0x100;
  uint16_t tilemapAdr = ppu->bgLayer[2].tilemapAdr + (((y >> tileBits) & 0x1f) << 5 | ((x >> tileBits) & 0x1f));
  if((x & tileHighBit) && ppu->bgLayer[2].tilemapWider) tilemapAdr += 0x400;
  if((y & tileHighBit) && ppu->bgLayer[2].tilemapHigher) tilemapAdr += ppu->bgLayer[2].tilemapWider ? 0x800 : 0x400;
  return ppu->vram[tilemapAdr & 0x7fff];
}

static int ppu_getPixelForBgLayer(Ppu* ppu, int x, int y, int layer, bool priority) {
  // figure out address of tilemap word and read it
  bool wideTiles = ppu->bgLayer[layer].bigTiles || ppu->mode == 5 || ppu->mode == 6;
  int tileBitsX = wideTiles ? 4 : 3;
  int tileHighBitX = wideTiles ? 0x200 : 0x100;
  int tileBitsY = ppu->bgLayer[layer].bigTiles ? 4 : 3;
  int tileHighBitY = ppu->bgLayer[layer].bigTiles ? 0x200 : 0x100;
  uint16_t tilemapAdr = ppu->bgLayer[layer].tilemapAdr + (((y >> tileBitsY) & 0x1f) << 5 | ((x >> tileBitsX) & 0x1f));
  if((x & tileHighBitX) && ppu->bgLayer[layer].tilemapWider) tilemapAdr += 0x400;
  if((y & tileHighBitY) && ppu->bgLayer[layer].tilemapHigher) tilemapAdr += ppu->bgLayer[layer].tilemapWider ? 0x800 : 0x400;
  uint16_t tile = ppu->vram[tilemapAdr & 0x7fff];
  // check priority, get palette
  if(((bool) (tile & 0x2000)) != priority) return 0; // wrong priority
  int paletteNum = (tile & 0x1c00) >> 10;
  // figure out position within tile
  int row = (tile & 0x8000) ? 7 - (y & 0x7) : (y & 0x7);
  int col = (tile & 0x4000) ? (x & 0x7) : 7 - (x & 0x7);
  int tileNum = tile & 0x3ff;
  if(wideTiles) {
    // if unflipped right half of tile, or flipped left half of tile
    if(((bool) (x & 8)) ^ ((bool) (tile & 0x4000))) tileNum += 1;
  }
  if(ppu->bgLayer[layer].bigTiles) {
    // if unflipped bottom half of tile, or flipped upper half of tile
    if(((bool) (y & 8)) ^ ((bool) (tile & 0x8000))) tileNum += 0x10;
  }
  // read tiledata, ajust palette for mode 0
  int bitDepth = bitDepthsPerMode[ppu->mode][layer];
  if(ppu->mode == 0) paletteNum += 8 * layer;
  // plane 1 (always)
  int paletteSize = 4;
  uint16_t plane1 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + row) & 0x7fff];
  int pixel = (plane1 >> col) & 1;
  pixel |= ((plane1 >> (8 + col)) & 1) << 1;
  // plane 2 (for 4bpp, 8bpp)
  if(bitDepth > 2) {
    paletteSize = 16;
    uint16_t plane2 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + 8 + row) & 0x7fff];
    pixel |= ((plane2 >> col) & 1) << 2;
    pixel |= ((plane2 >> (8 + col)) & 1) << 3;
  }
  // plane 3 & 4 (for 8bpp)
  if(bitDepth > 4) {
    paletteSize = 256;
    uint16_t plane3 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + 16 + row) & 0x7fff];
    pixel |= ((plane3 >> col) & 1) << 4;
    pixel |= ((plane3 >> (8 + col)) & 1) << 5;
    uint16_t plane4 = ppu->vram[(ppu->bgLayer[layer].tileAdr + ((tileNum & 0x3ff) * 4 * bitDepth) + 24 + row) & 0x7fff];
    pixel |= ((plane4 >> col) & 1) << 6;
    pixel |= ((plane4 >> (8 + col)) & 1) << 7;
  }
  // return cgram index, or 0 if transparent, palette number in bits 10-8 for 8-color layers
  return pixel == 0 ? 0 : paletteSize * paletteNum + pixel;
}

static void ppu_calculateMode7Starts(Ppu* ppu, int y) {
  // expand 13-bit values to signed values
  int hScroll = ((int16_t) (ppu->m7matrix[6] << 3)) >> 3;
  int vScroll = ((int16_t) (ppu->m7matrix[7] << 3)) >> 3;
  int xCenter = ((int16_t) (ppu->m7matrix[4] << 3)) >> 3;
  int yCenter = ((int16_t) (ppu->m7matrix[5] << 3)) >> 3;
  // do calculation
  int clippedH = hScroll - xCenter;
  int clippedV = vScroll - yCenter;
  clippedH = (clippedH & 0x2000) ? (clippedH | ~1023) : (clippedH & 1023);
  clippedV = (clippedV & 0x2000) ? (clippedV | ~1023) : (clippedV & 1023);
  if(ppu->bgLayer[0].mosaicEnabled && ppu->mosaicSize > 1) {
    y -= (y - ppu->mosaicStartLine) % ppu->mosaicSize;
  }
  uint8_t ry = ppu->m7yFlip ? 255 - y : y;
  ppu->m7startX = (
    ((ppu->m7matrix[0] * clippedH) & ~63) +
    ((ppu->m7matrix[1] * ry) & ~63) +
    ((ppu->m7matrix[1] * clippedV) & ~63) +
    (xCenter << 8)
  );
  ppu->m7startY = (
    ((ppu->m7matrix[2] * clippedH) & ~63) +
    ((ppu->m7matrix[3] * ry) & ~63) +
    ((ppu->m7matrix[3] * clippedV) & ~63) +
    (yCenter << 8)
  );
}

static int ppu_getPixelForMode7(Ppu* ppu, int x, int layer, bool priority) {
  uint8_t rx = ppu->m7xFlip ? 255 - x : x;
  int xPos = (ppu->m7startX + ppu->m7matrix[0] * rx) >> 8;
  int yPos = (ppu->m7startY + ppu->m7matrix[2] * rx) >> 8;
  bool outsideMap = xPos < 0 || xPos >= 1024 || yPos < 0 || yPos >= 1024;
  xPos &= 0x3ff;
  yPos &= 0x3ff;
  if(!ppu->m7largeField) outsideMap = false;
  uint8_t tile = outsideMap ? 0 : ppu->vram[(yPos >> 3) * 128 + (xPos >> 3)] & 0xff;
  uint8_t pixel = outsideMap && !ppu->m7charFill ? 0 : ppu->vram[tile * 64 + (yPos & 7) * 8 + (xPos & 7)] >> 8;
  if(layer == 1) {
    if(((bool) (pixel & 0x80)) != priority) return 0;
    return pixel & 0x7f;
  }
  return pixel;
}

static bool ppu_getWindowState(Ppu* ppu, int layer, int x) {
  if(!ppu->windowLayer[layer].window1enabled && !ppu->windowLayer[layer].window2enabled) {
    return false;
  }
  if(ppu->windowLayer[layer].window1enabled && !ppu->windowLayer[layer].window2enabled) {
    bool test = x >= ppu->window1left && x <= ppu->window1right;
    return ppu->windowLayer[layer].window1inversed ? !test : test;
  }
  if(!ppu->windowLayer[layer].window1enabled && ppu->windowLayer[layer].window2enabled) {
    bool test = x >= ppu->window2left && x <= ppu->window2right;
    return ppu->windowLayer[layer].window2inversed ? !test : test;
  }
  bool test1 = x >= ppu->window1left && x <= ppu->window1right;
  bool test2 = x >= ppu->window2left && x <= ppu->window2right;
  if(ppu->windowLayer[layer].window1inversed) test1 = !test1;
  if(ppu->windowLayer[layer].window2inversed) test2 = !test2;
  switch(ppu->windowLayer[layer].maskLogic) {
    case 0: return test1 || test2;
    case 1: return test1 && test2;
    case 2: return test1 != test2;
    case 3: return test1 == test2;
  }
  return false;
}

/* One pass over OAM builds, for every scanline, the set of sprites whose
 * y-range covers it — so the per-line evaluation only visits candidates
 * instead of rescanning all 128 entries 224 times a frame. Candidacy is a
 * function of OAM y bytes, the highOam size bits, OBSEL and SETINI alone;
 * writes to any of those clear objCacheValid. x, priority rotation and the
 * hardware's 32-sprite/34-tile limits are still applied per line, in the
 * exact order of the full scan, so the output is bit-identical. */
static void ppu_rebuildSpriteLineCache(Ppu *ppu) {
  memset(ppu->objLineCand, 0, sizeof(ppu->objLineCand));
  for (int s = 0; s < 128; s++) {
    uint8_t index = (uint8_t)(s * 2);
    uint8_t y = ppu->oam[index] >> 8;
    int spriteSize = spriteSizes[ppu->objSize][(ppu->highOam[index >> 3] >> ((index & 7) + 1)) & 1];
    int spriteHeight = ppu->objInterlace ? spriteSize / 2 : spriteSize;
    for (int row = 0; row < spriteHeight; row++) {
      uint8_t l = (uint8_t)(y + row);   /* same wraparound as (uint8)(line - y) < height */
      if (l < 240)
        ppu->objLineCand[l][s >> 5] |= 1u << (s & 31);
    }
  }
  ppu->objCacheValid = 1;
}

static bool ppu_evaluateSprites(Ppu* ppu, int line) {
  // TODO: iterate over oam normally to determine in-range sprites,
  //   then iterate those in-range sprites in reverse for tile-fetching
  // TODO: rectangular sprites, wierdness with sprites at -256
  if (!ppu->objCacheValid)
    ppu_rebuildSpriteLineCache(ppu);
  const uint32_t *cand = ppu->objLineCand[line];
  int spritesFound = 0;
  int tilesFound = 0;
  /* the full scan started at this sprite and wrapped through all 128;
   * visit the candidates in that same order: s0..127, then 0..s0-1 */
  int s0 = ppu->objPriority ? ((ppu->oamAdr & 0xfe) >> 1) : 0;
  for (int half = 0; half != 2; half++) {
    int lo = half ? 0 : s0, hi = half ? s0 : 128;
    for (int w = lo >> 5; w * 32 < hi; w++) {
      uint32_t bits = cand[w];
      if (w == lo >> 5)
        bits &= ~0u << (lo & 31);
      if (hi - w * 32 < 32)
        bits &= (1u << (hi & 31)) - 1;
      while (bits) {
        int s = w * 32 + __builtin_ctz(bits);
        bits &= bits - 1;
        uint8_t index = (uint8_t)(s * 2);
        uint8_t y = ppu->oam[index] >> 8;
        // check if the sprite is on this line and get the sprite size
        uint8_t row = line - y;
        int spriteSize = spriteSizes[ppu->objSize][(ppu->highOam[index >> 3] >> ((index & 7) + 1)) & 1];
        {
          // in y-range, get the x location, using the high bit as well
          int x = ppu->oam[index] & 0xff;
          x |= ((ppu->highOam[index >> 3] >> (index & 7)) & 1) << 8;
          if(x > 255) x -= 512;
          // if in x-range
          if(x > -spriteSize) {
            // break if we found 32 sprites already
            spritesFound++;
            if(spritesFound > 32) {
              ppu->rangeOver = true;
              goto done;
            }
            // update row according to obj-interlace
            if(ppu->objInterlace) row = row * 2 + (ppu->evenFrame ? 0 : 1);
            // get some data for the sprite and y-flip row if needed
            int oam1 = ppu->oam[index + 1];
            int objAdr = (oam1 & 0x100) ? ppu->objTileAdr2 : ppu->objTileAdr1;
            if(oam1 & 0x8000) row = spriteSize - 1 - row;
            // fetch all tiles in x-range
            int paletteBase = 0x80 + 16 * ((oam1 & 0xe00) >> 9);
            int prio = SPRITE_PRIO_TO_PRIO((oam1 & 0x3000) >> 12, (oam1 & 0x800) == 0);
            PpuZbufType z = paletteBase + (prio << 8);

            for(int col = 0; col < spriteSize; col += 8) {
              if(col + x > -8 && col + x < 256) {
                // break if we found 34 8*1 slivers already
                tilesFound++;
                if(tilesFound > 34) {
                  ppu->timeOver = true;
                  goto done;
                }
                // figure out which tile this uses, looping within 16x16 pages, and get it's data
                int usedCol = oam1 & 0x4000 ? spriteSize - 1 - col : col;
                int usedTile = ((((oam1 & 0xff) >> 4) + (row >> 3)) << 4) | (((oam1 & 0xf) + (usedCol >> 3)) & 0xf);
                uint16 *addr = &ppu->vram[(objAdr + usedTile * 16 + (row & 0x7)) & 0x7fff];
                PPU_PROBE_VRAM_ADR(objAdr + usedTile * 16 + (row & 0x7));
                uint32 plane = addr[0] | addr[8] << 16;
                uint32 chunky = PpuDecode4bpp(plane);
                // go over each pixel
                int px_left = IntMax(-(col + x + kPpuExtraLeftRight), 0);
                int px_right = IntMin(256 + kPpuExtraLeftRight - (col + x), 8);
                PpuZbufType *dst = ppu->objBuffer.data + col + x + px_left + kPpuExtraLeftRight;

                if (oam1 & 0x4000) {
                  chunky >>= px_left * 4;
                  for (int px = px_left; px < px_right; px++, dst++, chunky >>= 4) {
                    int pixel = chunky & 0xf;
                    if (pixel != 0 && (dst[0] & 0xff) == 0)
                      dst[0] = z + pixel;
                  }
                } else {
                  chunky <<= px_left * 4;
                  for (int px = px_left; px < px_right; px++, dst++, chunky <<= 4) {
                    int pixel = chunky >> 28;
                    if (pixel != 0 && (dst[0] & 0xff) == 0)
                      dst[0] = z + pixel;
                  }
                }

              }
            }
          }
        }
      }
    }
  }
done:
  return tilesFound != 0;
}

static uint16_t ppu_getVramRemap(Ppu* ppu) {
  uint16_t adr = ppu->vramPointer;
  switch(ppu->vramRemapMode) {
    case 0: return adr;
    case 1: return (adr & 0xff00) | ((adr & 0xe0) >> 5) | ((adr & 0x1f) << 3);
    case 2: return (adr & 0xfe00) | ((adr & 0x1c0) >> 6) | ((adr & 0x3f) << 3);
    case 3: return (adr & 0xfc00) | ((adr & 0x380) >> 7) | ((adr & 0x7f) << 3);
  }
  return adr;
}

uint8_t ppu_read(Ppu* ppu, uint8_t adr) {
  switch(adr) {
    case 0x04: case 0x14: case 0x24:
    case 0x05: case 0x15: case 0x25:
    case 0x06: case 0x16: case 0x26:
    case 0x08: case 0x18: case 0x28:
    case 0x09: case 0x19: case 0x29:
    case 0x0a: case 0x1a: case 0x2a: {
      return ppu->ppu1openBus;
    }
    case 0x34:
    case 0x35:
    case 0x36: {
      int result = ppu->m7matrix[0] * (ppu->m7matrix[1] >> 8);
      ppu->ppu1openBus = (result >> (8 * (adr - 0x34))) & 0xff;
      return ppu->ppu1openBus;
    }
    case 0x37: {
      // TODO: only when ppulatch is set
      ppu->hCount = ppu->snes->hPos / 4;
      ppu->vCount = ppu->snes->vPos;
      ppu->countersLatched = true;
      return ppu->snes->openBus;
    }
    case 0x38: {
      uint8_t ret = 0;
      if(ppu->oamInHigh) {
        ret = ppu->highOam[((ppu->oamAdr & 0xf) << 1) | ppu->oamSecondWrite];
        if(ppu->oamSecondWrite) {
          ppu->oamAdr++;
          if(ppu->oamAdr == 0) ppu->oamInHigh = false;
        }
      } else {
        if(!ppu->oamSecondWrite) {
          ret = ppu->oam[ppu->oamAdr] & 0xff;
        } else {
          ret = ppu->oam[ppu->oamAdr++] >> 8;
          if(ppu->oamAdr == 0) ppu->oamInHigh = true;
        }
      }
      ppu->oamSecondWrite = !ppu->oamSecondWrite;
      ppu->ppu1openBus = ret;
      return ret;
    }
    case 0x39: {
      uint16_t val = ppu->vramReadBuffer;
      if(!ppu->vramIncrementOnHigh) {
        ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
        ppu->vramPointer += ppu->vramIncrement;
      }
      ppu->ppu1openBus = val & 0xff;
      return val & 0xff;
    }
    case 0x3a: {
      uint16_t val = ppu->vramReadBuffer;
      if(ppu->vramIncrementOnHigh) {
        ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
        ppu->vramPointer += ppu->vramIncrement;
      }
      ppu->ppu1openBus = val >> 8;
      return val >> 8;
    }
    case 0x3b: {
      uint8_t ret = 0;
      if(!ppu->cgramSecondWrite) {
        ret = ppu->cgram[ppu->cgramPointer] & 0xff;
      } else {
        ret = ((ppu->cgram[ppu->cgramPointer++] >> 8) & 0x7f) | (ppu->ppu2openBus & 0x80);
      }
      ppu->cgramSecondWrite = !ppu->cgramSecondWrite;
      ppu->ppu2openBus = ret;
      return ret;
    }
    case 0x3c: {
      uint8_t val = 0;
      if(ppu->hCountSecond) {
        val = ((ppu->hCount >> 8) & 1) | (ppu->ppu2openBus & 0xfe);
      } else {
        val = ppu->hCount & 0xff;
      }
      ppu->hCountSecond = !ppu->hCountSecond;
      ppu->ppu2openBus = val;
      return val;
    }
    case 0x3d: {
      uint8_t val = 0;
      if(ppu->vCountSecond) {
        val = ((ppu->vCount >> 8) & 1) | (ppu->ppu2openBus & 0xfe);
      } else {
        val = ppu->vCount & 0xff;
      }
      ppu->vCountSecond = !ppu->vCountSecond;
      ppu->ppu2openBus = val;
      return val;
    }
    case 0x3e: {
      uint8_t val = 0x1; // ppu1 version (4 bit)
      val |= ppu->ppu1openBus & 0x10;
      val |= ppu->rangeOver << 6;
      val |= ppu->timeOver << 7;
      ppu->ppu1openBus = val;
      return val;
    }
    case 0x3f: {
      uint8_t val = 0x3; // ppu2 version (4 bit), bit 4: ntsc/pal
      val |= ppu->ppu2openBus & 0x20;
      val |= ppu->countersLatched << 6;
      val |= ppu->evenFrame << 7;
      ppu->countersLatched = false; // TODO: only when ppulatch is set
      ppu->hCountSecond = false;
      ppu->vCountSecond = false;
      ppu->ppu2openBus = val;
      return val;
    }
    default: {
      return ppu->snes->openBus;
    }
  }
}

void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val) {
//  if (adr != 24 && adr != 25)
//    printf("ppu_write(%d, %d)\n", adr, val);
  switch(adr) {
    case 0x00: {
      // TODO: oam address reset when written on first line of vblank, (and when forced blank is disabled?)
      ppu->brightness = val & 0xf;
      ppu->forcedBlank = val & 0x80;
      break;
    }
    case 0x01: {
      ppu->objSize = val >> 5;
      ppu->objTileAdr1 = (val & 7) << 13;
      ppu->objTileAdr2 = ppu->objTileAdr1 + (((val & 0x18) + 8) << 9);
      ppu->objCacheValid = 0;   /* sprite sizes moved */
      break;
    }
    case 0x02: {
      ppu->oamAdr = val;
      ppu->oamAdrWritten = ppu->oamAdr;
      ppu->oamInHigh = ppu->oamInHighWritten;
      ppu->oamSecondWrite = false;
      break;
    }
    case 0x03: {
      ppu->objPriority = val & 0x80;
      ppu->oamInHigh = val & 1;
      ppu->oamInHighWritten = ppu->oamInHigh;
      ppu->oamAdr = ppu->oamAdrWritten;
      ppu->oamSecondWrite = false;
      break;
    }
    case 0x04: {
      if(ppu->oamInHigh) {
        uint32_t high_index = ((ppu->oamAdr & 0xf) << 1) | ppu->oamSecondWrite;
        uint8_t *dst = &ppu->highOam[high_index];
#ifdef SNES_LINE_REUSE_PROBE
        if (*dst != val) {
          g_probe_oam_gen++;
          for (int s = high_index * 4; s < high_index * 4 + 4; s++)
            g_probe_oam_entry_gen[s]++;
        }
#endif
#ifdef SNES_LINE_CACHE
        if (*dst != val) {
          int first = high_index * 4;
          PpuLineCacheBump(&g_line_cache_oam_serial, &g_line_cache_oam_last[first]);
          for (int s = first + 1; s < first + 4; s++)
            g_line_cache_oam_last[s] = g_line_cache_oam_serial;
        }
#endif
        *dst = val;
        ppu->objCacheValid = 0;   /* size / x-high bits moved */
        if(ppu->oamSecondWrite) {
          ppu->oamAdr++;
          if(ppu->oamAdr == 0) ppu->oamInHigh = false;
        }
      } else {
        if(!ppu->oamSecondWrite) {
          ppu->oamBuffer = val;
        } else {
          uint16_t value = (val << 8) | ppu->oamBuffer;
#ifdef SNES_LINE_REUSE_PROBE
          if (ppu->oam[ppu->oamAdr] != value) {
            g_probe_oam_gen++;
            g_probe_oam_entry_gen[ppu->oamAdr >> 1]++;
          }
#endif
#ifdef SNES_LINE_CACHE
          if (ppu->oam[ppu->oamAdr] != value)
            PpuLineCacheBump(&g_line_cache_oam_serial,
                             &g_line_cache_oam_last[ppu->oamAdr >> 1]);
#endif
          ppu->oam[ppu->oamAdr++] = value;
          ppu->objCacheValid = 0;   /* a sprite may have moved vertically */
          if(ppu->oamAdr == 0) ppu->oamInHigh = true;
        }
      }
      ppu->oamSecondWrite = !ppu->oamSecondWrite;
      break;
    }
    case 0x05: {
      ppu->mode = val & 0x7;
      ppu->bg3priority = val & 0x8;
      ppu->bgLayer[0].bigTiles = val & 0x10;
      ppu->bgLayer[1].bigTiles = val & 0x20;
      ppu->bgLayer[2].bigTiles = val & 0x40;
      ppu->bgLayer[3].bigTiles = val & 0x80;
      break;
    }
    case 0x06: {
      // TODO: mosaic line reset specifics
      ppu->bgLayer[0].mosaicEnabled = val & 0x1;
      ppu->bgLayer[1].mosaicEnabled = val & 0x2;
      ppu->bgLayer[2].mosaicEnabled = val & 0x4;
      ppu->bgLayer[3].mosaicEnabled = val & 0x8;
      ppu->mosaicSize = (val >> 4) + 1;
      ppu->mosaicStartLine = 0;// ppu->snes->vPos;
      break;
    }
    case 0x07:
    case 0x08:
    case 0x09:
    case 0x0a: {
      ppu->bgLayer[adr - 7].tilemapWider = val & 0x1;
      ppu->bgLayer[adr - 7].tilemapHigher = val & 0x2;
      ppu->bgLayer[adr - 7].tilemapAdr = (val & 0xfc) << 8;
      break;
    }
    case 0x0b: {
      ppu->bgLayer[0].tileAdr = (val & 0xf) << 12;
      ppu->bgLayer[1].tileAdr = (val & 0xf0) << 8;
      break;
    }
    case 0x0c: {
      ppu->bgLayer[2].tileAdr = (val & 0xf) << 12;
      ppu->bgLayer[3].tileAdr = (val & 0xf0) << 8;
      break;
    }
    case 0x0d: {
      ppu->m7matrix[6] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      // fallthrough to normal layer BG-HOFS
    }
    case 0x0f:
    case 0x11:
    case 0x13: {
      ppu->bgLayer[(adr - 0xd) / 2].hScroll = ((val << 8) | (ppu->scrollPrev & 0xf8) | (ppu->scrollPrev2 & 0x7)) & 0x3ff;
      ppu->scrollPrev = val;
      ppu->scrollPrev2 = val;
      break;
    }
    case 0x0e: {
      ppu->m7matrix[7] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      // fallthrough to normal layer BG-VOFS
    }
    case 0x10:
    case 0x12:
    case 0x14: {
      ppu->bgLayer[(adr - 0xe) / 2].vScroll = ((val << 8) | ppu->scrollPrev) & 0x3ff;
      ppu->scrollPrev = val;
      break;
    }
    case 0x15: {
      if((val & 3) == 0) {
        ppu->vramIncrement = 1;
      } else if((val & 3) == 1) {
        ppu->vramIncrement = 32;
      } else {
        ppu->vramIncrement = 128;
      }
      ppu->vramRemapMode = (val & 0xc) >> 2;
      ppu->vramIncrementOnHigh = val & 0x80;
      break;
    }
    case 0x16: {
      ppu->vramPointer = (ppu->vramPointer & 0xff00) | val;
      ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
      break;
    }
    case 0x17: {
      ppu->vramPointer = (ppu->vramPointer & 0x00ff) | (val << 8);
      ppu->vramReadBuffer = ppu->vram[ppu_getVramRemap(ppu) & 0x7fff];
      break;
    }
    case 0x18: {
      // TODO: vram access during rendering (also cgram and oam)
      uint16_t vramAdr = ppu_getVramRemap(ppu);
      uint16_t *dst = &ppu->vram[vramAdr & 0x7fff];
      uint16_t value = (*dst & 0xff00) | val;
#ifdef SNES_LINE_REUSE_PROBE
      if (*dst != value) {
        g_probe_vram_gen++;
        g_probe_vram_page_gen[(vramAdr & 0x7fff) >> 6]++;
      }
#endif
#ifdef SNES_LINE_CACHE
      if (*dst != value)
        PpuLineCacheBump(&g_line_cache_vram_serial,
                         &g_line_cache_vram_last[(vramAdr & 0x7fff) >> kLineCacheVramShift]);
#endif
      *dst = value;
      if(!ppu->vramIncrementOnHigh) ppu->vramPointer += ppu->vramIncrement;
      break;
    }
    case 0x19: {
      uint16_t vramAdr = ppu_getVramRemap(ppu);
      uint16_t *dst = &ppu->vram[vramAdr & 0x7fff];
      uint16_t value = (*dst & 0x00ff) | (val << 8);
#ifdef SNES_LINE_REUSE_PROBE
      if (*dst != value) {
        g_probe_vram_gen++;
        g_probe_vram_page_gen[(vramAdr & 0x7fff) >> 6]++;
      }
#endif
#ifdef SNES_LINE_CACHE
      if (*dst != value)
        PpuLineCacheBump(&g_line_cache_vram_serial,
                         &g_line_cache_vram_last[(vramAdr & 0x7fff) >> kLineCacheVramShift]);
#endif
      *dst = value;
      if(ppu->vramIncrementOnHigh) ppu->vramPointer += ppu->vramIncrement;
      break;
    }
    case 0x1a: {
      ppu->m7largeField = val & 0x80;
      ppu->m7charFill = val & 0x40;
      ppu->m7yFlip = val & 0x2;
      ppu->m7xFlip = val & 0x1;
      break;
    }
    case 0x1b:
    case 0x1c:
    case 0x1d:
    case 0x1e: {
      ppu->m7matrix[adr - 0x1b] = (val << 8) | ppu->m7prev;
      ppu->m7prev = val;
      break;
    }
    case 0x1f:
    case 0x20: {
      ppu->m7matrix[adr - 0x1b] = ((val << 8) | ppu->m7prev) & 0x1fff;
      ppu->m7prev = val;
      break;
    }
    case 0x21: {
      ppu->cgramPointer = val;
      ppu->cgramSecondWrite = false;
      break;
    }
    case 0x22: {
      if(!ppu->cgramSecondWrite) {
        ppu->cgramBuffer = val;
      } else {
        uint16_t value = (val << 8) | ppu->cgramBuffer;
#ifdef SNES_LINE_REUSE_PROBE
        if (ppu->cgram[ppu->cgramPointer] != value) {
          g_probe_cgram_gen++;
          g_probe_cgram_entry_gen[ppu->cgramPointer]++;
        }
#endif
#ifdef SNES_LINE_CACHE
        if (ppu->cgram[ppu->cgramPointer] != value)
          PpuLineCacheBump(&g_line_cache_cgram_serial,
                           &g_line_cache_cgram_last[ppu->cgramPointer]);
#endif
        ppu->cgram[ppu->cgramPointer++] = value;
#ifdef PPU_RGB565
        ppu->paletteDirty = true;
#endif
      }
      ppu->cgramSecondWrite = !ppu->cgramSecondWrite;
      break;
    }
    case 0x23:
    case 0x24:
    case 0x25: {

      if (adr == 0x23)
        ppu->windowsel = (ppu->windowsel & ~0xff) | val;
      else if (adr == 0x24)
        ppu->windowsel = (ppu->windowsel & ~0xff00) | (val << 8);
      else if (adr == 0x25)
        ppu->windowsel = (ppu->windowsel & ~0xff0000) | (val << 16);

      ppu->windowLayer[(adr - 0x23) * 2].window1inversed = val & 0x1;
      ppu->windowLayer[(adr - 0x23) * 2].window1enabled = val & 0x2;
      ppu->windowLayer[(adr - 0x23) * 2].window2inversed = val & 0x4;
      ppu->windowLayer[(adr - 0x23) * 2].window2enabled = val & 0x8;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window1inversed = val & 0x10;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window1enabled = val & 0x20;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window2inversed = val & 0x40;
      ppu->windowLayer[(adr - 0x23) * 2 + 1].window2enabled = val & 0x80;
      break;
    }
    case 0x26: {
      ppu->window1left = val;
      break;
    }
    case 0x27: {
      ppu->window1right = val;
      break;
    }
    case 0x28: {
      ppu->window2left = val;
      break;
    }
    case 0x29: {
      ppu->window2right = val;
      break;
    }
    case 0x2a: {
      ppu->windowLayer[0].maskLogic = val & 0x3;
      ppu->windowLayer[1].maskLogic = (val >> 2) & 0x3;
      ppu->windowLayer[2].maskLogic = (val >> 4) & 0x3;
      ppu->windowLayer[3].maskLogic = (val >> 6) & 0x3;
      break;
    }
    case 0x2b: {
      ppu->windowLayer[4].maskLogic = val & 0x3;
      ppu->windowLayer[5].maskLogic = (val >> 2) & 0x3;
      break;
    }
    case 0x2c: {
      ppu->screenEnabled[0] = val;
      ppu->layer[0].mainScreenEnabled = val & 0x1;
      ppu->layer[1].mainScreenEnabled = val & 0x2;
      ppu->layer[2].mainScreenEnabled = val & 0x4;
      ppu->layer[3].mainScreenEnabled = val & 0x8;
      ppu->layer[4].mainScreenEnabled = val & 0x10;
      break;
    }
    case 0x2d: {
      ppu->screenEnabled[1] = val;
      ppu->layer[0].subScreenEnabled = val & 0x1;
      ppu->layer[1].subScreenEnabled = val & 0x2;
      ppu->layer[2].subScreenEnabled = val & 0x4;
      ppu->layer[3].subScreenEnabled = val & 0x8;
      ppu->layer[4].subScreenEnabled = val & 0x10;
      break;
    }
    case 0x2e: {
      ppu->screenWindowed[0] = val;
      ppu->layer[0].mainScreenWindowed = val & 0x1;
      ppu->layer[1].mainScreenWindowed = val & 0x2;
      ppu->layer[2].mainScreenWindowed = val & 0x4;
      ppu->layer[3].mainScreenWindowed = val & 0x8;
      ppu->layer[4].mainScreenWindowed = val & 0x10;
      break;
    }
    case 0x2f: {
      ppu->screenWindowed[1] = val;
      ppu->layer[0].subScreenWindowed = val & 0x1;
      ppu->layer[1].subScreenWindowed = val & 0x2;
      ppu->layer[2].subScreenWindowed = val & 0x4;
      ppu->layer[3].subScreenWindowed = val & 0x8;
      ppu->layer[4].subScreenWindowed = val & 0x10;
      break;
    }
    case 0x30: {
      ppu->directColor = val & 0x1;
      ppu->addSubscreen = val & 0x2;
      ppu->preventMathMode = (val & 0x30) >> 4;
      ppu->clipMode = (val & 0xc0) >> 6;
      break;
    }
    case 0x31: {
      ppu->subtractColor = val & 0x80;
      ppu->halfColor = val & 0x40;
      for(int i = 0; i < 6; i++) {
        ppu->mathEnabled[i] = val & (1 << i);
      }
      break;
    }
    case 0x32: {
      if(val & 0x80) ppu->fixedColorB = val & 0x1f;
      if(val & 0x40) ppu->fixedColorG = val & 0x1f;
      if(val & 0x20) ppu->fixedColorR = val & 0x1f;
      break;
    }
    case 0x33: {
      ppu->interlace = val & 0x1;
      if (ppu->objInterlace != (bool)(val & 0x2))
        ppu->objCacheValid = 0;   /* sprite heights halve/double */
      ppu->objInterlace = val & 0x2;
      ppu->overscan = val & 0x4;
      ppu->pseudoHires = val & 0x8;
      ppu->m7extBg = val & 0x40;
      break;
    }
    default: {
      break;
    }
  }
}

int PpuGetCurrentRenderScale(Ppu *ppu, uint32_t render_flags) {
  return 1;
}
