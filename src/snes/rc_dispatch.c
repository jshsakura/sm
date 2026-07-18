/* rc_dispatch.c — per-bank hash dispatch for the static recompiler.
 *
 * Portable: used by both the M7 rig (flat-linked, no XIP) and the device
 * (XIP blob cached in flash). The caller (main_snes.c on device, rig entry
 * on host) supplies the rc_addrs/rc_fns pointers at activation time.
 *
 * Hash: Knuth multiplicative, per-bank open-addressing, LF~0.5.
 * SMW (8371 sites, 7 banks): ~60 KB total, fits DTCM.
 * Rig-measured cost: ~4.57M insn/frame (−42.3% vs interpreter). */
#include "rc_dispatch.h"
#include <stdlib.h>
#include <string.h>

bool g_rc_active = false;
void (**g_rc_fns)(Cpu *) = NULL;

typedef struct { uint16_t pc; uint16_t id; } rc_entry_t;

static rc_entry_t *rc_hash[256];
static int rc_hash_mask[256];

void rc_dispatch_init(const uint32_t *addrs, uint32_t nsites,
                      void (**fns)(Cpu *)) {
  rc_dispatch_reset();
  g_rc_fns = fns;

  int counts[256] = {0};
  for (uint32_t i = 0; i < nsites; i++)
    counts[addrs[i] >> 16]++;

  for (int b = 0; b < 256; b++) {
    if (!counts[b]) continue;
    int sz = 1;
    while (sz < counts[b] * 2) sz <<= 1;
    rc_hash[b] = malloc(sz * sizeof(rc_entry_t));
    memset(rc_hash[b], 0, sz * sizeof(rc_entry_t));
    rc_hash_mask[b] = sz - 1;
  }

  for (uint32_t i = 0; i < nsites; i++) {
    uint8_t b = addrs[i] >> 16;
    uint16_t pc = addrs[i] & 0xffff;
    int mask = rc_hash_mask[b];
    uint32_t h = (pc * 2654435761u) & mask;
    while (rc_hash[b][h].id)
      h = (h + 1) & mask;
    rc_hash[b][h].pc = pc;
    rc_hash[b][h].id = (uint16_t)(i + 1);
  }

  g_rc_active = true;
}

void rc_dispatch_reset(void) {
  for (int b = 0; b < 256; b++) {
    free(rc_hash[b]);
    rc_hash[b] = NULL;
    rc_hash_mask[b] = 0;
  }
  g_rc_fns = NULL;
  g_rc_active = false;
}

uint16_t rc_dispatch_lookup(uint8_t bank, uint16_t pc) {
  int mask = rc_hash_mask[bank];
  if (!mask) return 0;
  rc_entry_t *ht = rc_hash[bank];
  uint32_t h = (pc * 2654435761u) & mask;
  for (;;) {
    uint16_t eid = ht[h].id;
    if (!eid) return 0;
    if (ht[h].pc == pc) return eid;
    h = (h + 1) & mask;
  }
}
