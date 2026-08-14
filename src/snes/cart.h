
#ifndef CART_H
#define CART_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct Cart Cart;

#include "snes.h"

typedef struct Dsp1 Dsp1;

struct Cart {
  Snes* snes;
  uint8_t type;

  uint8_t* rom;
  uint32_t romSize;
  uint32_t romMask;   /* nonzero only when romSize is a power of 2 (fast path) */
  /* Whether snes_cpuRead's fetch-page cache can serve this cart at all.
   * Decided once at load so the hot path never re-asks; see cart_setRomSize. */
  uint8_t romPageOk;
  /* Host base of each mapped bank: LoROM indexes it by bank&0x7f in 32 KB
   * steps, HiROM by bank&0x3f in 64 KB. Built once at load so the page install
   * in snes_cpuRead is a table load and an add -- no romMask, no cart_fold(),
   * and above all no CALL inside the ITCM bus reader. */
  uint8_t* bankBase[128];
  uint8_t* ram;
  uint32_t ramSize;

  /* DSP-1 coprocessor (Mario Kart / Pilotwings class). NULL for normal carts —
   * every added branch below is behind this test, so plain games cost nothing. */
  Dsp1* dsp1;
};

// TODO: how to handle reset & load? (especially where to init ram)

Cart* cart_init(Snes* snes);
void cart_setRomSize(Cart* cart, int size);
void cart_attachDsp1(Cart* cart);   /* call after cart_load for DSP carts */
void cart_free(Cart* cart);
void cart_reset(Cart* cart); // will reset special chips etc, general reading is set up in load
void cart_load(Cart* cart, int type, uint8_t* rom, int romSize, int ramSize); // TODO: figure out how to handle (battery, cart-chips etc)
uint8_t cart_read(Cart* cart, uint8_t bank, uint16_t adr);
void cart_write(Cart* cart, uint8_t bank, uint16_t adr, uint8_t val);
void cart_saveload(Cart *cart, SaveLoadFunc *func, void *ctx);
#endif
