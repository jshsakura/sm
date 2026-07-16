/* NMI-wait spin skip — exact-replay (SNES_SPIN_SKIP builds).
 *
 * The spin probe measured Zelda 81% / SMW 76% of gameplay opcodes inside pure
 * WRAM/DP-flag wait loops (`spin: LDA $12 / BEQ spin`). Those iterations are
 * semantic no-ops: registers bit-identical each pass, no writes, no IO reads —
 * only the NMI handler can change the polled byte, and no handler can run inside
 * a run_dots span (events fire only at span boundaries). So inside a span the
 * loop provably cannot exit, and each iteration can be replayed without the
 * interpreter: charge the recorded cycle pattern, advance pc along the recorded
 * opcode ring, and let the SAME bulk-consume code chunk the dots — identical
 * hPos steps, identical apuCatchupCycles FMA sequence, identical cpuCyclesLeft
 * arithmetic. Bit-identical state, minus the interpreter work.
 *
 * This is the ONE implementation: the device port (main_snes.c) and the host
 * gate harness (tools/snes_spin) both compile this file, so what the harness
 * proves is what the device runs. Gate: skip off vs on must produce identical
 * state and audio hashes (tools/snes_spin/run_skip.sh).
 *
 * Runtime auto-gate (address-agnostic, NO per-game list — Korean-patched and
 * rom-hacked carts behave identically): games that never spin (TMNT-class)
 * would pay the learner's per-opcode cost for nothing, so if an observation
 * window ends with (almost) no replayed ops the learner parks itself and
 * retries later — a cart that starts spinning after a loading phase is picked
 * back up within a window. */
#ifndef SNES_SPIN_SKIP_H
#define SNES_SPIN_SKIP_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Cpu Cpu;

#define SPIN_PMAX 8   /* longest loop body (opcodes) we replay */
#define SPIN_LR   16  /* learning ring: recent real opcode calls */

typedef struct {
  /* adopted pattern (the loop being replayed) */
  uint32_t pc[SPIN_PMAX];
  uint8_t  charge[SPIN_PMAX];
  int      len, idx;
  bool     on;
  /* purity sequence numbers, bumped by the cpu.c hooks */
  uint64_t write_seq;
  uint64_t io_seq;
  /* learning ring of recent (pc24, ccl-charge, seqs, regs) real calls */
  struct { uint32_t pc; uint8_t charge; uint64_t w, io, r1, r2; } lr[SPIN_LR];
  int      lr_h, lr_n;
  /* auto-gate: learning enabled + observation-window bookkeeping */
  bool     gate_on;
  uint32_t win_real, win_virtual;
  uint32_t park_frames;   /* frames left to sit out after gating off */
  /* stats (read by the port for diagnostics) */
  uint64_t ops_real, ops_virtual;
} SpinSkip;

extern SpinSkip g_spin;

/* cpu.c write hook: any store breaks purity. */
static inline void spin_hook_write(void) { g_spin.write_seq++; }

/* cpu.c read hook: classify a read as side-effect-free (opcode-adjacent, WRAM,
 * ROM) or an IO read that breaks purity ($21xx/$42xx/$43xx: reading APU ports or
 * HVBJOY moves observable state). Address-agnostic, exactly the harness rule. */
void spin_hook_read(Cpu *cpu, uint32_t adr);

/* Record one real opcode call (pre-call pc24, total ccl charge, pre-call regs);
 * learns/keeps/drops the pattern. dispatched = an interrupt entered
 * cpu_runOpcode instead of the opcode at pc24. */
void spin_note(uint32_t pc24, uint8_t charge, int dispatched,
               uint64_t r1, uint64_t r2);

/* Once per emulated frame: runs the observation-window auto-gate. */
void spin_frame_tick(void);

void spin_reset(void);

#endif
