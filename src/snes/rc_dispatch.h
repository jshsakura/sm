/* rc_dispatch.h — per-ROM static recompiler dispatch layer.
 *
 * When activated (g_rc_active == true), cpu_runOpcode consults a per-bank
 * open-addressing hash table to find a native site function for the current
 * PC. If found, the site runs instead of the interpreter. If not found
 * (0.01% of opcodes on SMW), the interpreter handles it.
 *
 * The dispatch table is built at activation time from the rc blob's
 * rc_addrs[] array (24-bit kpc values). The site function pointers
 * (rc_fns[]) live in the XIP blob; the hash table lives in DTCM.
 *
 * Activation is ROM-specific: main_snes.c CRC-checks the ROM and, if it
 * matches a pre-compiled rc blob, caches the blob into flash and calls
 * rc_dispatch_init. Non-matching ROMs leave g_rc_active false — zero
 * overhead (one never-taken branch in cpu_runOpcode).
 *
 * See rc_dispatch_hash.c (M7 rig) for the measured dispatch cost:
 * ~4.57M insn/frame on SMW (−42.3% vs interpreter's 7.92M). */
#ifndef RC_DISPATCH_H
#define RC_DISPATCH_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu.h"

/* Runtime flag: when true, cpu_runOpcode uses rc dispatch. Set by
 * rc_dispatch_activate(), cleared by rc_dispatch_reset(). */
extern bool g_rc_active;

/* Build the hash dispatch table from the given site addresses + function
 * pointers. Allocates per-bank hash tables (DTCM via malloc). After this
 * call, g_rc_active is true and cpu_runOpcode will use the rc path.
 *
 *   addrs  — array of 24-bit kpc values (k<<16 | pc), one per site
 *   nsites — number of sites
 *   fns    — array of function pointers, one per site (in the XIP blob) */
void rc_dispatch_init(const uint32_t *addrs, uint32_t nsites,
                      void (**fns)(Cpu *));

/* Clear the dispatch state and free the hash tables. g_rc_active = false. */
void rc_dispatch_reset(void);

/* Look up a (bank, pc) pair in the dispatch table.
 * Returns site_id (1-based) if found, 0 if not (interpreter fallback).
 * Called from cpu_runOpcode on every opcode when g_rc_active is true. */
uint16_t rc_dispatch_lookup(uint8_t bank, uint16_t pc);

/* Call site id (1-based). Defined as a thin wrapper so cpu.c doesn't need
 * to know about the function pointer table layout. */
static inline void rc_dispatch_call(uint16_t id, Cpu *cpu) {
  extern void (**g_rc_fns)(Cpu *);
  g_rc_fns[id - 1](cpu);
}

#endif /* RC_DISPATCH_H */
