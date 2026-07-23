
#ifndef CPU_H
#define CPU_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "saveload.h"

typedef struct Cpu Cpu;

struct Cpu {
  // reference to memory handler, for reading//writing
  void* mem;
  uint8_t memType; // used to define which type mem is
  // registers
  uint16_t a;
  uint16_t x;
  uint16_t y;
  uint16_t sp;
  uint16_t pc;
  uint16_t dp; // direct page (D)
  uint8_t k; // program bank (PB)
  uint8_t db; // data bank (B)
  // flags
  bool c;
  bool z;
  bool v;
  bool n;
  bool i;
  bool d;
  bool xf;
  bool mf;
  bool e;
  // interrupts
  bool irqWanted;
  bool nmiWanted;
  // power state (WAI/STP)
  bool waiting;
  bool stopped;
  // internal use
  uint8_t cyclesUsed; // indicates how many cycles an opcode used
  uint16_t spBreakpoint;
  bool in_emu;
};

extern struct Cpu *g_cpu;
bool HookedFunctionRts(int is_long);

Cpu* cpu_init(void* mem, int memType);
void cpu_free(Cpu* cpu);
void cpu_reset(Cpu* cpu);
int cpu_runOpcode(Cpu* cpu);
#ifdef SNES_THUMB2_CPU
/* The C interpreter exposed as oracle/fallback for the Thumb-2 dispatcher. */
int cpu_runOpcode_c(Cpu* cpu);
/* Thumb-2 fast path. The caller (try path) has already fetched the opcode and
   charged cyclesPerOpcode[opcode]; the step path fetches it itself. Either way,
   on entry pc points just past the opcode byte. Each native handler performs
   exactly the operand/data fetches and extra cycle charges its opcode semantics
   require (e.g. branches, immediate ALU/loads, and REP/SEP consume their
   operands at pc), so a handler is correct on BOTH paths: pc consistently
   points at the operand. Returns 1 if handled (cpu state mutated in place), 0
   to fall back to C, which then runs cpu_doOpcode on the already-fetched byte. */
int snes_thumb2_try(Cpu* cpu, uint8_t opcode);
/* Stage 2 fetch-dispatch entry. Fetches EXACTLY ONE opcode by calling the real
   snes_cpuRead(mem, (k<<16)|pc), increments the 16-bit pc once, charges
   cyclesUsed from snes_cycles_per_opcode, and runs the handler. Returns -1 if
   handled, or the opcode byte (0..255) if unsupported — the caller then calls
   cpu_doOpcode on that already-fetched byte with no second fetch or cycle charge. */
int snes_thumb2_step(Cpu* cpu);
#endif
uint8_t cpu_getFlags(Cpu *cpu);
void cpu_setFlags(Cpu *cpu, uint8_t val);
void cpu_saveload(Cpu *cpu, SaveLoadFunc *func, void *ctx);
#endif
