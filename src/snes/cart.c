#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../types.h"
#include "cart.h"
#include "snes.h"
#include "dsp1_hle.h"

/* Coprocessor dispatch (Dsp1, Cx4) exists ONLY in the generic SNES core.
 * SM and SMW compile this same cart.c with -DHEADLESS and no
 * -DGNW_SNES_CORE, and neither their source lists nor their redefines files
 * know the dsp1 and cx4 symbol families: a reference from here would be an undefined
 * symbol -- or, once two overlays share RAM_EMU, a silent alias onto another
 * core's data. The Cart *fields* stay in cart.h for every build so
 * sizeof(Cart) is identical across cores; only the code that touches the
 * chips is compiled out. */
#ifdef GNW_SNES_CORE
#include "cx4_hle.h"

/* Weak fallbacks so build scripts that list snes/*.c files explicitly and
 * predate dsp1_hle.c still link; without the strong definitions a DSP cart
 * just behaves as before (status reads 0). Harness globs pick up the real
 * implementation automatically. */
__attribute__((weak)) void dsp1_reset(Dsp1* d) { (void)d; }
__attribute__((weak)) uint8_t dsp1_readDR(Dsp1* d) { (void)d; return 0; }
__attribute__((weak)) void dsp1_writeDR(Dsp1* d, uint8_t v) { (void)d; (void)v; }
__attribute__((weak)) uint8_t dsp1_readSR(Dsp1* d) { (void)d; return 0; }

/* sizeof(Dsp1) is unknown when only weak stubs are linked, so allocation lives
 * here behind the same weak mechanism: the strong version in dsp1_hle.c
 * allocates for real. */
__attribute__((weak)) Dsp1* dsp1_alloc(void) { return NULL; }
__attribute__((weak)) uint32_t dsp1_size(void) { return 0; }
#endif

#ifdef GNW_SNES_CORE
#include <assert.h>
/* Device save-RAM. Writable, so it can't be XIP'd from flash like the ROM is —
 * but the ~81 KB DTCM heap can't malloc a 32 KB SRAM either. Park it in a static
 * buffer, which lands in .overlay_snes_bss (RAM_EMU, 724 KB — the roomy region
 * where WRAM already lives), not the heap. 0x8000 is the largest SRAM a plain
 * LoROM/HiROM cart declares; coprocessor carts (which can want more) are rejected
 * at load, and the assert in cart_load() catches anything that slips past. */
static uint8_t gnw_cart_sram[0x8000];
#endif


/* SNES carts mirror their ROM across the address space. A power-of-2 image needs
 * only a mask, but 1.5 MB / 3 MB / 6 MB images are common (37% of a real library)
 * and for those the top of the range folds back onto the last power-of-2 chunk —
 * exactly what the cart's chip-select decoding does on hardware. Precompute a
 * mask when we can and fall back to bsnes's fold otherwise, so the hot path stays
 * a single AND for most games. */
static uint32_t cart_fold(uint32_t addr, uint32_t size) {
  if (size == 0) return 0;
  uint32_t base = 0, mask = 1u << 31;
  while (addr >= size) {
    while (!(addr & mask)) mask >>= 1;
    addr -= mask;
    if (size > mask) { size -= mask; base += mask; }
  }
  return base + addr;
}

static inline uint32_t cart_romIndex(Cart* cart, uint32_t addr) {
  if (cart->romMask) return addr & cart->romMask;      /* power of 2: one AND */
  return cart_fold(addr, (uint32_t)cart->romSize);
}

/* snes_cpuRead's fetch-page cache used to require cart->romMask -- a
 * power-of-two ROM -- because that is the only size whose index is one AND.
 * Every other size fell through to snes_read() -> cart_read() ->
 * cart_readLorom(), four calls and a full classification chain, on EVERY
 * opcode fetch. "Every other size" is not exotic: a 24 Mbit cartridge is 3 MB,
 * and Super Metroid took the slow path for 30,463 of its 34,314 bus reads a
 * frame.
 *
 * cart_romIndex() already folds a non-power-of-two ROM, and the fold is
 * piecewise linear in chunks that are powers of two no smaller than the lowest
 * set bit of romSize. So the base of each BANK can be precomputed once and the
 * offset added -- which is cheaper than the mask it replaces, and works for any
 * size. That is what these two tables are.
 *
 * Which banks have ROM below $8000, transcribed from cart_read{Lo,Hi}rom's own
 * decode rather than re-derived. The fast path used to require adr >= $8000,
 * which is the whole of a LoROM bank but only HALF of a HiROM one -- so a HiROM
 * cartridge fetched half its opcodes through the slow path no matter what.
 *
 * The banks that must NOT be claimed are the ones something else decodes:
 * $00-$3f/$80-$bf below $8000 is WRAM and MMIO; LoROM SRAM sits at $70-$7d and
 * $f0-$ff; HiROM SRAM (and its DSP window) at $00-$3f, $6000-$7fff -- all of
 * which are below $40 after masking and so already excluded. */
static void cart_buildBankLowRom(Cart* cart) {
  for(int raw = 0; raw < 256; raw++) {
    uint8_t b7 = (uint8_t)(raw & 0x7f);
    uint8_t ok = 0;
    if(raw != 0x7e && raw != 0x7f) {            /* WRAM banks, decoded earlier */
      if(cart->type == 1) {
        int isSram = (((raw >= 0x70 && raw < 0x7e) || raw >= 0xf0) &&
                      cart->ramSize > 0);
        ok = (!isSram && b7 >= 0x40) ? 1 : 0;
      } else if(cart->type == 2) {
        ok = (b7 >= 0x40) ? 1 : 0;
      }
    }
    cart->bankLowRom[raw] = cart->romPageOk ? ok : 0;
  }
}

static void cart_buildBankBases(Cart* cart) {
  for(int b = 0; b < 128; b++) cart->bankBase[b] = cart->rom;
  if(cart->romPageOk) {
    if(cart->type == 1) {
      for(int b = 0; b < 128; b++)
        cart->bankBase[b] = cart->rom + cart_romIndex(cart, (uint32_t)b << 15);
    } else {
      for(int b = 0; b < 64; b++)
        cart->bankBase[b] = cart->rom + cart_romIndex(cart, (uint32_t)b << 16);
    }
  }
  cart_buildBankLowRom(cart);
}

void cart_setRomSize(Cart* cart, int size) {
  cart->romSize = size;
  cart->romMask = (size > 0 && (size & (size - 1)) == 0) ? (uint32_t)(size - 1) : 0;
  /* Decided once, here, so snes_cpuRead's hot path never asks a question whose
   * answer cannot change -- one load and a branch, exactly what the
   * `cart->romMask` test it replaces already cost.
   *
   * An earlier version asked a helper per read whether the cart was cacheable
   * and was told NULL again every time: +1.8% instructions a frame on one such
   * dump. Ask once.
   *
   * A whole number of BANKS, because the bank-base table is built once per
   * bank: the fold has to be linear across one, which needs every chunk of its
   * decomposition -- and so the lowest set bit of romSize -- to be at least a
   * bank wide.
   *
   * A HiROM bank is 64 KB and a LoROM bank is 32 KB, and that difference is not
   * academic: demanding 64 KB of both took the cache away from a 32 KB LoROM
   * cart, which is a cart that HAD one before any of this work (romMask is set
   * for any power of two). 2,069 of the 2,074 cartridges in the reference
   * library qualify; the five that do not are dumps carrying a 512-byte
   * remainder, and they keep the slow path. */
  const uint32_t bankGran = (cart->type == 2) ? 0xffffu : 0x7fffu;
  cart->romPageOk = (cart->rom != NULL && size > 0 &&
                     ((uint32_t)size & bankGran) == 0 &&
                     (cart->type == 1 || cart->type == 2)) ? 1 : 0;
  /* Tables here too, so a caller that only ever sets the size (main_sm.c,
   * the sm harness) still gets a consistent cart. cart_load() calls this
   * BEFORE it assigns cart->ramSize, so it rebuilds them at the end -- see
   * the comment there; that ordering is what made bank $70 read as ROM. */
  cart_buildBankBases(cart);
}

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr);
static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);

Cart* cart_init(Snes* snes) {
  Cart* cart = malloc(sizeof(Cart));
  cart->snes = snes;
  cart->type = 0;
  cart->rom = NULL;
  cart->romSize = 0;
  cart->romMask = 0;
  cart->ram = NULL;
  cart->ramSize = 0;
  cart->dsp1 = NULL;
  /* Same reason as dsp1 above: the DTCM heap is NOT zeroed on device, so a
   * Cx4-less cart would dispatch reads through a garbage pointer. */
  cart->cx4 = NULL;
  /* malloc'd, and on the device the DTCM heap is not zeroed -- an uninitialised
   * romPageOk would send snes_cpuRead through a bankBase[] full of garbage
   * before the first cart_load(). */
  cart->romPageOk = 0;
  memset(cart->bankLowRom, 0, sizeof(cart->bankLowRom));
  for(int b = 0; b < 128; b++) cart->bankBase[b] = NULL;
  return cart;
}

#ifdef GNW_SNES_CORE
void cart_attachCx4(Cart* cart) {
  if (cart->cx4 == NULL) {
    /* calloc, not malloc: mirrors dsp1_alloc(), and zeroes the whole struct
     * (cx4_init() only clears ->ram, leaving ->version whatever the heap
     * had). A failed 8 KB alloc must not take the quiet path either. */
    cart->cx4 = calloc(1, sizeof(struct Cx4));
    if (cart->cx4 == NULL) {
      printf("cart_attachCx4: Cx4 alloc failed\n");
      return;
    }
  }
  cx4_init(cart->cx4);
  // Cx4 relies on LoROM fast path for ROM fetches since its MMIO is below $8000
  // so we don't need to punch out cart->bankBase for Cx4 like DSP-1 does.
}
#endif

#ifdef GNW_SNES_CORE
void cart_attachDsp1(Cart* cart) {
  if (cart->dsp1 == NULL) cart->dsp1 = dsp1_alloc();
  if (cart->dsp1) {
    dsp1_reset(cart->dsp1);
    /* LoROM boards decode the DSP at banks $30-$3f, $8000-$ffff — inside the
     * range snes_cpuRead's ROM fast path claims.
     *
     * This used to read `if (cart->type == 1) cart->romMask = 0;`, which turned
     * the fast path AND the fetch-page cache off for the WHOLE 24-bit bus so
     * that sixteen banks would decode correctly. Every opcode fetch on every
     * LoROM DSP cartridge then went snes_cpuRead -> snes_read -> cart_read ->
     * cart_readLorom, four calls and a full classification chain, three of them
     * out of ITCM into the RAM_EMU overlay. A device PC sample of Pilotwings
     * charged cart_read 12.1% and snes_read 9.8% of the frame for it.
     *
     * snes_cpuRead excludes the window itself now, and it does the test only
     * where a page tag is INSTALLED -- a page hit never sees it. HiROM DSP
     * boards need nothing: their window is $00-$1f:6000-7fff, below the
     * $8000 the fast path requires.
     *
     * SNES_DSP_FASTPATH=0 restores the old behaviour, for the A/B arm. */
#if !SNES_DSP_FASTPATH
    if (cart->type == 1) cart->romMask = 0;
#endif
  }
}
#endif /* GNW_SNES_CORE */

void cart_free(Cart* cart) {
  if (cart->dsp1) free(cart->dsp1);
  if (cart->cx4) free(cart->cx4);
  free(cart);
}

void cart_reset(Cart* cart) {
  //if(cart->ramSize > 0 && cart->ram != NULL) memset(cart->ram, 0, cart->ramSize); // for now
#ifdef GNW_SNES_CORE
  if (cart->dsp1) dsp1_reset(cart->dsp1);
  if (cart->cx4) cx4_init(cart->cx4);
#endif
}

void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx) {
  func(ctx, cart->ram, cart->ramSize);
#ifdef GNW_SNES_CORE
  /* DSP carts append the chip state (plain data, versioned via its first
   * field). Normal carts write exactly what they always did, so existing
   * savestates stay byte-compatible. */
  if (cart->dsp1) func(ctx, cart->dsp1, dsp1_size());
  if (cart->cx4) func(ctx, cart->cx4, sizeof(struct Cx4));
#endif
}

void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize) {
  cart->type = type;
#ifdef GNW_SNES_CORE
  // Device: neither the ROM nor the SRAM touches the ~81 KB heap. `rom` is the
  // flash-mapped image used in place (never freed); the SRAM lives in a static
  // RAM_EMU buffer, not malloc. Nothing here is heap-owned, so nothing is freed.
  cart->rom = rom;
  cart_setRomSize(cart, romSize);
  if(ramSize > 0) {
    /* A header can claim more SRAM than the 32 KB static buffer (64/128 KB
     * carts exist). Clamp instead of assert-BSOD: the power-of-2 clamp keeps
     * every `& (ramSize-1)` mask valid, so oversized carts see a mirrored
     * 32 KB -- degraded but running, exactly what an undersized SRAM chip
     * does on a repro board. */
    if(ramSize > (int)sizeof(gnw_cart_sram)) ramSize = (int)sizeof(gnw_cart_sram);
    cart->ram = gnw_cart_sram;
    memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;
  /* AFTER ramSize. cart_setRomSize() above built the bank tables, and
   * cart_buildBankLowRom() has to know whether banks $70-$7d are SRAM -- which
   * at that point they were not, because cart->ramSize was still whatever the
   * previous cartridge left. Super Metroid then served its 8 KB save RAM out of
   * the ROM page cache and locked up around frame 400, with the fast path
   * answering 0xcc where the bus says 0x00. */
  cart_buildBankBases(cart);
#else
  if(cart->rom != NULL) free(cart->rom);
  if(cart->ram != NULL) free(cart->ram);
  cart->rom = malloc(romSize);
  cart_setRomSize(cart, romSize);
  if(ramSize > 0) {
    cart->ram = malloc(ramSize);
    memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;
  memcpy(cart->rom, rom, romSize);
  cart_buildBankBases(cart);   /* AFTER ramSize -- see the device branch above */
#endif
}

uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr) {
  switch(cart->type) {
    case 0: 
      assert(0);
      return cart->snes->openBus;
    case 1: return cart_readLorom(cart, bank, adr);
    case 2: return cart_readHirom(cart, bank, adr);
  }
  assert(0);
  return cart->snes->openBus;
}

void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  switch(cart->type) {
    case 0: break;
    case 1: cart_writeLorom(cart, bank, adr, val); break;
    case 2: cart_writeHirom(cart, bank, adr, val); break;
  }
}

void DumpCpuHistory();

static uint8_t cart_readLorom(Cart* cart, uint8_t bank, uint16_t adr) {
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    return cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)];
  }
  bank &= 0x7f;
#ifdef GNW_SNES_CORE
  // Cx4 on LoROM boards: banks 00-3f, adr 6000-7fff
  if(cart->cx4 && bank < 0x40 && adr >= 0x6000 && adr < 0x8000) {
    return cx4_read(cart->cx4, adr);
  }
  // DSP-1 on LoROM boards: banks 30-3f, DR 8000-bfff / SR c000-ffff
  if(cart->dsp1 && bank >= 0x30 && bank < 0x40 && adr >= 0x8000) {
    return adr < 0xc000 ? dsp1_readDR(cart->dsp1) : dsp1_readSR(cart->dsp1);
  }
#endif
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[cart_romIndex(cart, ((uint32_t)bank << 15) | (adr & 0x7fff))];
  }
#ifdef GNW_SNES_CORE
  /* General-purpose core: an unmapped cart-region read is OPEN BUS on real
   * hardware (last value on the bus), and commercial games do stray reads
   * there in normal play. Crashing here is a homebrew-development aid, not
   * emulation -- keep it for the sm/zelda3 dev builds below, never here. */
  return cart->snes->openBus;
#else
  printf("While trying to read from 0x%x\n", bank << 16 | adr);
  DumpCpuHistory();
  Die("The game crashed in cart_readLorom");
  return cart->snes->openBus;
#endif
}

static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  /* Same decode order as cart_readLorom: SRAM first, then coprocessor
   * windows, then nothing (LoROM writes outside SRAM/chips are ignored).
   * This used to test the Cx4 window BEFORE SRAM and to use `bank > 0xf0`
   * where the read path uses `>= 0xf0` -- bank $f0 wrote to nothing while
   * its reads hit SRAM. */
  if(((bank >= 0x70 && bank < 0x7e) || bank >= 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
    return;
  }
#ifdef GNW_SNES_CORE
  bank &= 0x7f;
  // Cx4 on LoROM boards: banks 00-3f, adr 6000-7fff
  if(cart->cx4 && bank < 0x40 && adr >= 0x6000 && adr < 0x8000) {
    cx4_write(cart->cx4, adr, val, cart->rom, cart->romSize);
    return;
  }
  // DSP-1 on LoROM boards: banks 30-3f, adr 8000-ffff
  if(cart->dsp1 && bank >= 0x30 && bank < 0x40 && adr >= 0x8000) {
    if (adr < 0xc000) dsp1_writeDR(cart->dsp1, val);
  }
#endif
}

static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
#ifdef GNW_SNES_CORE
  // DSP-1 on HiROM boards: banks 00-1f, DR 6000-6fff / SR 7000-7fff.
  // SRAM decode moves up to banks 20-3f (matching real HiROM+DSP boards).
  if(cart->dsp1 && bank < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    return adr < 0x7000 ? dsp1_readDR(cart->dsp1) : dsp1_readSR(cart->dsp1);
  }
#endif
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[cart_romIndex(cart, (((uint32_t)(bank & 0x3f)) << 16) | adr)];
  }
#ifdef GNW_SNES_CORE
  /* General-purpose core: unmapped reads are open bus, same reasoning as
   * cart_readLorom above. Mario Kart reads $80:4A78 (bank 0 after masking,
   * $4400-$5FFF system-area hole) ~40 frames into the title screen -- real
   * hardware shrugs; this assert was a device BSOD (host builds compile
   * asserts out with -DNDEBUG, which is why every host run sailed past it). */
  return cart->snes->openBus;
#else
  assert(0);
  return cart->snes->openBus;
#endif
}

static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
#ifdef GNW_SNES_CORE
  if(cart->dsp1 && bank < 0x20 && adr >= 0x6000 && adr < 0x8000) {
    if (adr < 0x7000) dsp1_writeDR(cart->dsp1, val);
    return;
  }
#endif
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}
