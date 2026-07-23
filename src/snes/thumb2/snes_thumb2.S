@ Cortex-M7 Thumb-2 65816 engine -- Stage 0 guardrail.
@
@ This file proves the build path end to end: the .S rule produces a Thumb-2
@ object, the AAPCS symbol links, and the compile-checked offset header
@ (cpu_thumb2_offsets.h) resolves inside assembly. It executes no opcode and
@ changes no behavior; the dispatcher wiring lands in Stage 1.
@
@ Build: the Makefile.common snes_asm_obj_prereq_gen rule runs the file through
@ the C preprocessor (capital .S), so #include "cpu_thumb2_offsets.h" resolves
@ next to this source. -mcpu=cortex-m7 -mthumb is inherited from CFLAGS/MCU.

#include "cpu_thumb2_offsets.h"

        .syntax unified
        .cpu    cortex-m7
        .thumb

        .section .text.snes_thumb2_probe,"ax",%progbits
        .align  2
        .global snes_thumb2_probe
        .type   snes_thumb2_probe,%function
        .thumb_func
@ int snes_thumb2_probe(Cpu *cpu);  -- r0 = cpu
@ Returns 0, but first dereferences CPU_PC through the checked offset to prove
@ the offset header is live in this translation unit (the ldrh is elided only
@ if the compiler/assembler can prove it dead; with a volatile barrier it stays
@ so a disassembler sees the field access).
snes_thumb2_probe:
        ldrh    r1, [r0, #CPU_PC]          @ offset header resolves to #14
        movs    r0, #0
        bx      lr
        .size   snes_thumb2_probe, .-snes_thumb2_probe

        .section        .note.GNU-stack,"",%progbits
