
#ifndef SNES_H
#define SNES_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Snes Snes;

/* A LoROM DSP-1 board decodes the chip at banks $30-$3f (mirrored at $b0-$bf),
 * $8000-$ffff. snes_cpuRead's ROM fast path and its fetch-page cache claim
 * everything at $8000 and above, so that window has to come out of them.
 *
 * SNES_DSP_FASTPATH=1 takes it out with one test, evaluated only where a page
 * tag is installed. =0 is the old way: clear cart->romMask, which takes the
 * fast path away from the whole cartridge. Kept as the A/B arm. */
/* SNES_LINE_HIRQ=1 lets snes_run_line() keep its fast path when an H-timer is
 * armed, by splitting the line at the one dot the timer matches instead of
 * handing the whole line back to the dot loop. Only the native ports call
 * snes_run_line (Core/Src/porting/sm/main_sm.c); the SNES core runs
 * run_frame_events/run_dots and is untouched either way. */
#ifndef SNES_LINE_HIRQ
#define SNES_LINE_HIRQ 1
#endif

#ifndef SNES_DSP_FASTPATH
#define SNES_DSP_FASTPATH 1
#endif
/* SNES_ROMPAGE_FOLD=1 lets snes_cpuRead's fetch-page cache serve a cartridge
 * whose size is not a power of two -- 3 MB, 1.5 MB, 2.5 MB, most of the real
 * library. =0 is the old behaviour, where those carts had no cache and every
 * opcode fetch walked snes_read -> cart_read -> cart_read{Lo,Hi}rom. */
#ifndef SNES_ROMPAGE_FOLD
#define SNES_ROMPAGE_FOLD 1
#endif
#if SNES_ROMPAGE_FOLD
#define SNES_ROM_PAGE_OK(cart) ((cart)->romPageOk)
#else
#define SNES_ROM_PAGE_OK(cart) ((cart)->romMask)
#endif

/* SNES_ROMPAGE_LOW=1 also serves $0000-$7fff of the banks where that is ROM --
 * half of every HiROM bank, and all of LoROM's $40-$7d -- straight out of the
 * bank table, from the point where the $8000 test has already failed.
 *
 * OFF BY DEFAULT, by device measurement, and the numbers are worth keeping
 * because the instruction count says the opposite. Rig, 1200 frames, hashes
 * bit-identical throughout: Final Fantasy VI -4.5%, Chrono Trigger -2.0%,
 * Seiken Densetsu 2 -0.8%, Dark Law -2.6% instructions a frame -- and Super
 * Metroid +1.1%, Zelda 3 +0.8%, because a LoROM cart pays the test on every
 * slow read and almost never collects.
 *
 * On hardware, savestate play scenes, bracketed:
 *
 *   Chrono Trigger   58.06 -> 60.45 emulated fps, 20.0 -> 24.3 drawn (+21%)
 *   Final Fantasy VI 58.00 -> 55.80 emulated fps, 14.8 -> 14.0 drawn (-5.5%)
 *   A Link to the Past 61.66 -> 61.40,            21.2 -> 21.0 drawn (-1%)
 *
 * Two of three lose. An earlier variant that also INSTALLED a page tag for
 * those reads was worse still on FF6 (-6.5% drawn) while Chrono gained 36%: one
 * cache entry, the opcode stream at $8000+, and FF6's data in the bottom half
 * of the same banks, so every data read evicted the fetch page. Removing the
 * install fixed the thrash and FF6 still lost -- so the cost is the branch
 * itself, in the hottest function in ITCM, which is the shape this core has
 * lost to every time.
 *
 * =2 IS THAT DIAGNOSIS, TESTED: the same serve, with the test moved out of ITCM
 * into snes_read() -- the overlay call that was already happening. snes_cpuRead
 * is then byte-for-byte what it is at =0 (disassembled from both; only branch
 * targets move). Rig, 23 cartridges, every hash bit-identical:
 *
 *   Final Fantasy VI  -3.4%   Dark Law          -2.2%
 *   Chrono Trigger    -1.6%   JB The Super Bass -1.4%
 *   Seiken Densetsu 2 -1.1%   every LoROM cart  within +-0.03%
 *
 * The LoROM penalty is gone, which is what the diagnosis predicted. But =2
 * taxes the cartridges that cannot be page-cached AT ALL -- a dump whose size
 * is not a multiple of 64 KB, where romPageOk is 0 and bankLowRom is all
 * zeroes. Every read on such a cart is a slow read, so it pays the test ~34,000
 * times a frame and never collects: Jim Power +1.7%, JoJo's Bizarre Adventure
 * +2.2%.
 *
 * The way out of that is not a cheaper test, it is making those carts
 * cacheable: a 512-entry table keyed by 8 KB PAGE rather than by bank would
 * fold at page granularity and could mark the individual straddling pages
 * uncacheable. Not built.
 *
 * Nothing here is on by default until it has a device measurement -- =1's
 * instruction count preferred it and the console did not. */
#ifndef SNES_ROMPAGE_LOW
#define SNES_ROMPAGE_LOW 0
#endif
#if SNES_ROMPAGE_LOW == 1 && SNES_ROMPAGE_FOLD
#define SNES_BANK_LOW_ROM(cart, bank) ((cart)->bankLowRom[(bank)])
#else
#define SNES_BANK_LOW_ROM(cart, bank) 0
#endif
#if SNES_DSP_FASTPATH
#define SNES_DSP_LOROM_WINDOW(cart, bank) \
  ((cart)->dsp1 && (cart)->type == 1 && (uint8_t)(((bank) & 0x7f) - 0x30) < 0x10u)
#else
#define SNES_DSP_LOROM_WINDOW(cart, bank) 0
#endif

#include "cpu.h"
#include "apu.h"
#include "dma.h"
#include "ppu.h"
#include "cart.h"
#include "input.h"
#include "saveload.h"

struct Snes {
  Cpu* cpu;
  Apu* apu;
  Ppu* ppu, *snes_ppu, *my_ppu;
  Dma* dma;
  Cart* cart;
  Input *input1;
  Input *input2;
  // input
  bool debug_cycles;
  bool debug_apu_cycles;
  bool disableRender;
  uint8_t runningWhichVersion;

  // ram
  uint32_t ramAdr;
  uint8_t *ram;
  uint8_t padx[4];

  // frame timing
  uint16_t hPos;
  uint16_t vPos;
  uint32_t frames;
  // cpu handling
  uint8_t cpuCyclesLeft;
  uint8_t cpuMemOps;
  uint8_t padpad[2];
  double apuCatchupCycles;
  uint32_t apuDotsAccum;   /* integer dot accumulator — converted to double in snes_catchupApu to avoid per-dot VCVT+VMUL */
  // nmi / irq
  bool hIrqEnabled;
  bool vIrqEnabled;
  bool nmiEnabled;
  uint16_t hTimer;
  uint16_t vTimer;
  bool inNmi;
  bool inIrq;
  bool inVblank;
  // joypad handling
  uint16_t portAutoReadX[4]; // as read by auto-joypad read
  bool autoJoyRead;
  uint16_t autoJoyTimer; // times how long until reading is done
  bool ppuLatch;
  // multiplication/division
  uint8_t multiplyA;
  uint16_t multiplyResult;
  uint16_t divideA;
  uint16_t divideResult;
  // misc
  bool fastMem;
  uint8_t openBus;
  /* ROM fetch-page cache for snes_cpuRead(). Deliberately declared AFTER
   * openBus: snes_saveload() serializes the byte range hPos..openBus, so
   * anything past it stays out of the savestate -- which is what we want,
   * these are a derived host pointer and its tag, not emulated state, and a
   * savestate must never carry a host address across a load. */
  const uint8_t *romPageBase;
  uint32_t romPageTag;
};

Snes* snes_init(uint8_t *ram);
void snes_free(Snes* snes);
void snes_reset(Snes* snes, bool hard);
void snes_runFrame(Snes* snes);
// used by dma, cpu
uint8_t snes_readBBus(Snes* snes, uint8_t adr);
void snes_writeBBus(Snes* snes, uint8_t adr, uint8_t val);
uint8_t snes_read(Snes* snes, uint32_t adr);
void snes_write(Snes* snes, uint32_t adr, uint8_t val);
uint8_t snes_cpuRead(Snes* snes, uint32_t adr);
void snes_cpuWrite(Snes* snes, uint32_t adr, uint8_t val);
// debugging
void snes_debugCycle(Snes* snes, bool* cpuNext, bool* spcNext);

void snes_handle_pos_stuff(Snes *snes);

// snes_other.c functions:

bool snes_loadRom(Snes* snes, const uint8_t* data, int length);
void snes_setPixels(Snes* snes, uint8_t* pixelData);
void snes_setSamples(Snes* snes, int16_t* sampleData, int samplesPerFrame);
void snes_saveload(Snes *snes, SaveLoadFunc *func, void *ctx);
/* One scanline of dot-clock events in one call — see snes.c. */
void snes_run_line(Snes *snes);
uint8_t snes_readBBusOrg(Snes *snes, uint8_t adr);
void snes_catchupApu(Snes *snes);

extern int snes_frame_counter;
#endif
