#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../types.h"
#include "cart.h"
#include "snes.h"

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

void cart_setRomSize(Cart* cart, int size) {
  cart->romSize = size;
  cart->romMask = (size > 0 && (size & (size - 1)) == 0) ? (uint32_t)(size - 1) : 0;
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
  return cart;
}

void cart_free(Cart* cart) {
  free(cart);
}

void cart_reset(Cart* cart) {
  //if(cart->ramSize > 0 && cart->ram != NULL) memset(cart->ram, 0, cart->ramSize); // for now
}

void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx) {
  func(ctx, cart->ram, cart->ramSize);
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
    assert(ramSize <= (int)sizeof(gnw_cart_sram));
    cart->ram = gnw_cart_sram;
    memset(cart->ram, 0, ramSize);
  } else {
    cart->ram = NULL;
  }
  cart->ramSize = ramSize;
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
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[cart_romIndex(cart, ((uint32_t)bank << 15) | (adr & 0x7fff))];
  }
  printf("While trying to read from 0x%x\n", bank << 16 | adr);
  DumpCpuHistory();
  Die("The game crashed in cart_readLorom");
  return cart->snes->openBus;
}

static void cart_writeLorom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  if(((bank >= 0x70 && bank < 0x7e) || bank > 0xf0) && adr < 0x8000 && cart->ramSize > 0) {
    // banks 70-7e and f0-ff, adr 0000-7fff
    cart->ram[(((bank & 0xf) << 15) | adr) & (cart->ramSize - 1)] = val;
  }
}

static uint8_t cart_readHirom(Cart* cart, uint8_t bank, uint16_t adr) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    return cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)];
  }
  if(adr >= 0x8000 || bank >= 0x40) {
    // adr 8000-ffff in all banks or all addresses in banks 40-7f and c0-ff
    return cart->rom[cart_romIndex(cart, (((uint32_t)(bank & 0x3f)) << 16) | adr)];
  }
  assert(0);
  return cart->snes->openBus;
}

static void cart_writeHirom(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val) {
  bank &= 0x7f;
  if(bank < 0x40 && adr >= 0x6000 && adr < 0x8000 && cart->ramSize > 0) {
    // banks 00-3f and 80-bf, adr 6000-7fff
    cart->ram[(((bank & 0x3f) << 13) | (adr & 0x1fff)) & (cart->ramSize - 1)] = val;
  }
}
