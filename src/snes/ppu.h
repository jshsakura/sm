#ifndef PPU_H
#define PPU_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Ppu Ppu;

#include "snes.h"

typedef struct BgLayer {
  uint16_t hScroll;
  uint16_t vScroll;
  bool tilemapWider;
  bool tilemapHigher;
  uint16_t tilemapAdr;
  uint16_t tileAdr;
  bool bigTiles;
  bool mosaicEnabled;
} BgLayer;

enum {
  kPpuXPixels = 256,
  kPpuExtraLeftRight = 0,
};

/* Frameskip. The launcher drops video frames when it cannot hold 60 Hz, but the
 * SNES renders line by line from inside the frame's timing loop, so there is no
 * outer draw call to skip. Set this and the line renderer does the timing and
 * the sprite evaluation (the game can read range/time-over) but not the pixels,
 * which is where the work is. */
extern bool g_ppu_skip_render;

typedef uint16_t PpuZbufType;

// ClearBackdrop() fills these through a *(uint64*) cast. On ARM that compiles to
// STRD, which faults unless the address is word-aligned — and PpuZbufType only
// asks for 2. Left to the compiler these land at offset 0x702 inside Ppu, which
// is 2 mod 4, and the device takes a UsageFault on the first rendered line. The
// alignment the cast assumes has to be stated, not hoped for.
typedef struct PpuPixelPrioBufs {
  // This holds the prio in the upper 8 bits and the color in the lower 8 bits.
  PpuZbufType data[kPpuXPixels];
} __attribute__((aligned(8))) PpuPixelPrioBufs;

enum {
  kPpuRenderFlags_NewRenderer = 1,
  // Render mode7 upsampled by 4x4
  kPpuRenderFlags_4x4Mode7 = 2,
  // Use 240 height instead of 224
  kPpuRenderFlags_Height240 = 4,
  // Disable sprite render limits
  kPpuRenderFlags_NoSpriteLimits = 8,
};



typedef struct Layer {
  bool mainScreenEnabled;
  bool subScreenEnabled;
  bool mainScreenWindowed;
  bool subScreenWindowed;
} Layer;

typedef struct WindowLayer {
  bool window1enabled;
  bool window2enabled;
  bool window1inversed;
  bool window2inversed;
  uint8_t maskLogic;
} WindowLayer;

struct Ppu {
  Snes* snes;
  // vram access
#ifdef TARGET_GNW
  /* 64 KB of VRAM would dominate this struct, and the struct has to live in the
   * RAM_EMU overlay pool that also holds the game's code. Point at ITC RAM
   * instead (see ppu_init) — same trick the zelda3 G&W port uses. */
  uint16_t *vram;
#else
  uint16_t vram[0x8000];
#endif
  uint16_t vramPointer;
  bool vramIncrementOnHigh;
  uint16_t vramIncrement;
  uint8_t vramRemapMode;
  uint16_t vramReadBuffer;
  // cgram access
  uint16_t cgram[0x100];
  uint8_t cgramPointer;
  bool cgramSecondWrite;
  uint8_t cgramBuffer;
  // oam access
  uint16_t oam[0x100];
  uint8_t highOam[0x20];
  uint8_t oamAdr;
  uint8_t oamAdrWritten;
  bool oamInHigh;
  bool oamInHighWritten;
  bool oamSecondWrite;
  uint8_t oamBuffer;
  // object/sprites
  bool objPriority;
  uint16_t objTileAdr1;
  uint16_t objTileAdr2;
  uint8_t objSize;
  uint8_t objPixelBufferXX[256]; // line buffers
  uint8_t objPriorityBufferXX[256];
  bool timeOver;
  bool rangeOver;
  bool objInterlace;
  // background layers
  BgLayer bgLayer[4];
  uint8_t scrollPrev;
  uint8_t scrollPrev2;
  uint8_t mosaicSize;
  uint8_t mosaicStartLine;
  // layers
  Layer layer[5];
  // mode 7
  int16_t m7matrix[8]; // a, b, c, d, x, y, h, v
  uint8_t m7prev;
  bool m7largeField;
  bool m7charFill;
  bool m7xFlip;
  bool m7yFlip;
  bool m7extBg;
  // mode 7 internal
  int32_t m7startX;
  int32_t m7startY;
  // windows
  WindowLayer windowLayer[6];
  uint8_t window1left;
  uint8_t window1right;
  uint8_t window2left;
  uint8_t window2right;
  // color math
  uint8_t clipMode;
  uint8_t preventMathMode;
  bool addSubscreen;
  bool subtractColor;
  bool halfColor;
  bool mathEnabled[6];
  uint8_t fixedColorR;
  uint8_t fixedColorG;
  uint8_t fixedColorB;
  // settings
  bool forcedBlank;
  uint8_t brightness;
  uint8_t mode;
  bool bg3priority;
  bool evenFrame;
  bool pseudoHires;
  bool overscan;
  bool frameOverscan; // if we are overscanning this frame (determined at 0,225)
  bool interlace;
  bool frameInterlace; // if we are interlacing this frame (determined at start vblank)
  bool directColor;
  // latching
  uint16_t hCount;
  uint16_t vCount;
  bool hCountSecond;
  bool vCountSecond;
  bool countersLatched;
  uint8_t ppu1openBus;
  uint8_t ppu2openBus;
  // pixel buffer (xbgr)
  // times 2 for even and odd frame
  uint8_t pixelbuffer_placeholder;

  uint32_t windowsel;
  uint8_t extraLeftCur, extraRightCur, extraLeftRight;
  uint8_t screenEnabled[2];
  uint8_t screenWindowed[2];
  uint8_t mosaicEnabled;
  uint8_t lastBrightnessMult;
  bool lineHasSprites;
  PpuPixelPrioBufs bgBuffers[2];
  PpuPixelPrioBufs objBuffer;
  uint32_t renderPitch;
  uint8_t *renderBuffer;
  uint8_t brightnessMult[32 + 31];
  uint8_t brightnessMultHalf[32 * 2];
  uint8_t mosaicModulo[kPpuXPixels];

};

Ppu* ppu_init(Snes* snes);
void ppu_free(Ppu* ppu);
void ppu_copy(Ppu *ppu, Ppu *ppu_src);
void ppu_reset(Ppu* ppu);
bool ppu_checkOverscan(Ppu* ppu);
void ppu_handleVblank(Ppu* ppu);
void ppu_runLine(Ppu* ppu, int line);
uint8_t ppu_read(Ppu* ppu, uint8_t adr);
void ppu_write(Ppu* ppu, uint8_t adr, uint8_t val);
void ppu_saveload(Ppu *ppu, SaveLoadFunc *func, void *ctx);
void PpuBeginDrawing(Ppu *ppu, uint8_t *pixels, size_t pitch, uint32_t render_flags);

int PpuGetCurrentRenderScale(Ppu *ppu, uint32_t render_flags);

#endif
