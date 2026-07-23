@ Cortex-M7 Thumb-2 65816 engine -- Stage 2 direct fetch/dispatch bridge.
@
@ snes_thumb2_try(Cpu *cpu, uint8_t opcode) -> int
@   r0 = Cpu*, r1 = opcode
@   returns 1 if the opcode was handled (Cpu state updated in place), 0 if not.
@
@ Contract: the caller has ALREADY performed the single opcode fetch and charged
@ cyclesPerOpcode[opcode]; on entry pc points just past the opcode byte. Each
@ native handler performs exactly the operand/data fetches and extra cycle
@ charges its opcode semantics require, so a handler is correct on BOTH the
@ snes_thumb2_try path (caller-owned fetch) and the snes_thumb2_step path
@ (self-fetch): pc consistently points at the operand. Handled families:
@   no-operand: CLC SEC CLI SEI CLV CLD SED NOP
@              TAX TAY TXA TYA TSX TXS TCD TDC TCS TSC TXY TYX
@              INA INX INY DEA DEX DEY XBA
@   accumulator shifts: ASL A LSR A ROL A ROR A
@   relative branches (one operand byte at pc, taken-cycle per cpu_doBranch):
@              BPL BMI BVC BVS BRA BCC BCS BNE BEQ
@   immediate ALU/load: ORA AND EOR LDY LDX LDA
@   status masks: REP SEP
@ Every width-sensitive opcode honours the M/X/E flags exactly as cpu_doOpcode
@ does. Unsupported opcodes return 0 and leave *cpu untouched, so the C
@ dispatcher falls through to cpu_doOpcode(cpu, opcode) on the fetched byte.
@
@ Build path: Makefile.common snes_asm_obj_prereq_gen runs this file through
@ the C preprocessor (capital .S), so #include "cpu_thumb2_offsets.h" resolves
@ next to this source; SNES_C_INCLUDES adds the wider search path. The object
@ is renamed via snes_redefines (cpu_runOpcode -> gsnes__cpu_runOpcode etc.),
@ which is exactly what the C side is renamed to as well, so the BLX from the
@ Thumb dispatcher to the assembly resolves at link. AAPCS-TPCS: caller-saved
@ r0-r3 may be clobbered; callee-saved r4-r11 are preserved.

#include "cpu_thumb2_offsets.h"

        .syntax unified
        .cpu    cortex-m7
        .thumb

@ ---------------------------------------------------------------------------
@ setZN helper. Expects r4 = Cpu*, r2 = value already masked to the relevant
@ width (byte: 0..0xff, word: 0..0xffff), r3 = 1 for byte mode, 0 for word.
@ Writes CPU_Z (0/1) and CPU_N (0/1) -- normalised _Bool, matching what the C
@ compiler emits for `cpu->z = ... ; cpu->n = ...`.
        .macro SETZN
        cmp     r2, #0
        ite     eq
        moveq   r3, #1
        movne   r3, #0
        strb    r3, [r4, #CPU_Z]
        cmp     r1, #0                  @ r1 holds byte-mode flag at call sites
        bne     8f
        lsr     r3, r2, #15             @ word mode: n = bit15 (value < 0x10000)
        strb    r3, [r4, #CPU_N]
        b       9f
8:      lsr     r3, r2, #7              @ byte mode: n = bit7 (value < 0x100)
        strb    r3, [r4, #CPU_N]
9:
        .endm

@ Load a bool field (0/1) into r3 for width tests. Field offset in \off.
        .macro LDB r, off
        ldrb    \r, [r4, #(\off)]
        .endm

@ Inline one raw operand fetch.  This deliberately is a macro, not a callable
@ local helper: the contained BL may overwrite LR because the outer entry saved
@ its caller LR in the common prologue.  A BL-called helper containing this BL
@ would instead lose its own return address.  r6 carries all live state across
@ the C call (AAPCS callee-saved).
        .macro IMM_FETCH_BYTE
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ uint16 PC wrap; K never carries
        bl      snes_cpuRead
        uxtb    r0, r0
        .endm

@ Dense O(1) opcode dispatch.  Table entries carry the Thumb bit because the
@ local handler labels are not function symbols and therefore do not acquire
@ it automatically.
        .macro DISPATCH_ENTRY op
        .if \op == 0x18
        .word   .Lop18 + 1
        .elseif \op == 0x38
        .word   .Lop38 + 1
        .elseif \op == 0x58
        .word   .Lop58 + 1
        .elseif \op == 0x78
        .word   .Lop78 + 1
        .elseif \op == 0xb8
        .word   .Lopb8 + 1
        .elseif \op == 0xd8
        .word   .Lopd8 + 1
        .elseif \op == 0xf8
        .word   .Lopf8 + 1
        .elseif \op == 0xea
        .word   .Lopea + 1
        .elseif \op == 0x9a
        .word   .Lop9a + 1
        .elseif \op == 0x1b
        .word   .Lop1b + 1
        .elseif \op == 0x5b
        .word   .Lop5b + 1
        .elseif \op == 0x7b
        .word   .Lop7b + 1
        .elseif \op == 0x3b
        .word   .Lop3b + 1
        .elseif \op == 0x8a
        .word   .Lop8a + 1
        .elseif \op == 0x98
        .word   .Lop98 + 1
        .elseif \op == 0xaa
        .word   .Lopaa + 1
        .elseif \op == 0xa8
        .word   .Lopa8 + 1
        .elseif \op == 0xba
        .word   .Lopba + 1
        .elseif \op == 0x9b
        .word   .Lop9b + 1
        .elseif \op == 0xbb
        .word   .Lopbb + 1
        .elseif \op == 0x1a
        .word   .Lop1a + 1
        .elseif \op == 0x3a
        .word   .Lop3a + 1
        .elseif \op == 0xe8
        .word   .Lope8 + 1
        .elseif \op == 0xca
        .word   .Lopca + 1
        .elseif \op == 0xc8
        .word   .Lopc8 + 1
        .elseif \op == 0x88
        .word   .Lop88 + 1
        .elseif \op == 0xeb
        .word   .Lopeb + 1
        .elseif \op == 0x0a
        .word   .Lop0a + 1
        .elseif \op == 0x4a
        .word   .Lop4a + 1
        .elseif \op == 0x2a
        .word   .Lop2a + 1
        .elseif \op == 0x6a
        .word   .Lop6a + 1
        .elseif \op == 0x10
        .word   .Lop10 + 1
        .elseif \op == 0x30
        .word   .Lop30 + 1
        .elseif \op == 0x50
        .word   .Lop50 + 1
        .elseif \op == 0x70
        .word   .Lop70 + 1
        .elseif \op == 0x80
        .word   .Lop80 + 1
        .elseif \op == 0x90
        .word   .Lop90 + 1
        .elseif \op == 0xb0
        .word   .Lopb0 + 1
        .elseif \op == 0xd0
        .word   .Lopd0 + 1
        .elseif \op == 0xf0
        .word   .Lopf0 + 1
        .elseif \op == 0x09
        .word   .Lop09 + 1
        .elseif \op == 0x29
        .word   .Lop29 + 1
        .elseif \op == 0x49
        .word   .Lop49 + 1
        .elseif \op == 0xa0
        .word   .Lopa0 + 1
        .elseif \op == 0xa2
        .word   .Lopa2 + 1
        .elseif \op == 0xa9
        .word   .Lopa9 + 1
        .elseif \op == 0xc2
        .word   .Lopc2 + 1
        .elseif \op == 0xe2
        .word   .Lope2 + 1
        .else
        .word   .L_noteb + 1
        .endif
        .endm

@ A named placement section lets the linker collect this engine into ITCM only
@ when SNES_THUMB2_CPU=1 actually supplies the object as a link input.  Do not
@ name the object explicitly in the linker script: that breaks clean flag-off
@ links and can pull a stale flag-on object into an incremental flag-off build.
        .section .itcm_snes_interp.thumb2,"ax",%progbits
        .align  2
        .global snes_thumb2_try
        .type   snes_thumb2_try,%function
        .thumb_func
snes_thumb2_try:
        push    {r4, r5, r6, lr}
        mov     r4, r0                  @ r4 = Cpu* (callee-saved across handlers)
        movs    r5, #0                  @ return contract: try (1/0)
        uxtb    r6, r1
        b       .Ldispatch

@ Full single-opcode entry used by cpu_runOpcode().  Keep the public C pre-work
@ (stop/WAI/IRQ/NMI/RC) outside; this leaf owns exactly one opcode fetch, the
@ 16-bit PC increment, base-cycle charge, and direct/fallback decision.
        .global snes_thumb2_step
        .type   snes_thumb2_step,%function
        .thumb_func
snes_thumb2_step:
        push    {r4, r5, r6, lr}
        mov     r4, r0
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ uint16_t wrap matches cpu->pc++
        bl      snes_cpuRead
        uxtb    r6, r0                  @ preserve opcode across dispatch
        movw    r2, #:lower16:snes_cycles_per_opcode
        movt    r2, #:upper16:snes_cycles_per_opcode
        ldrb    r3, [r2, r6]
        strb    r3, [r4, #CPU_CYCLESUSED]
        movs    r5, #1                  @ return contract: step (-1/opcode)

.Ldispatch:
        adr     r2, .Ldispatch_table
        ldr     r3, [r2, r6, lsl #2]
        bx      r3

        .align  2
.Ldispatch_table:
        .set    .Ldispatch_op, 0
        .rept   256
        DISPATCH_ENTRY .Ldispatch_op
        .set    .Ldispatch_op, .Ldispatch_op + 1
        .endr

        @ --- flag operations (no width dependence, no ZN update) ---
        cmp     r1, #0x18               @ CLC
        bne     .L_not18
.Lop18:
        movs    r2, #0
        strb    r2, [r4, #CPU_C]
        b       .L_handled
.L_not18:
        cmp     r1, #0x38               @ SEC
        bne     .L_not38
.Lop38:
        movs    r2, #1
        strb    r2, [r4, #CPU_C]
        b       .L_handled
.L_not38:
        cmp     r1, #0x58               @ CLI
        bne     .L_not58
.Lop58:
        movs    r2, #0
        strb    r2, [r4, #CPU_I]
        b       .L_handled
.L_not58:
        cmp     r1, #0x78               @ SEI
        bne     .L_not78
.Lop78:
        movs    r2, #1
        strb    r2, [r4, #CPU_I]
        b       .L_handled
.L_not78:
        cmp     r1, #0xb8               @ CLV
        bne     .L_notb8
.Lopb8:
        movs    r2, #0
        strb    r2, [r4, #CPU_V]
        b       .L_handled
.L_notb8:
        cmp     r1, #0xd8               @ CLD
        bne     .L_notd8
.Lopd8:
        movs    r2, #0
        strb    r2, [r4, #CPU_D]
        b       .L_handled
.L_notd8:
        cmp     r1, #0xf8               @ SED
        bne     .L_notf8
.Lopf8:
        movs    r2, #1
        strb    r2, [r4, #CPU_D]
        b       .L_handled
.L_notf8:
        cmp     r1, #0xea               @ NOP
        bne     .L_notea
.Lopea:
        b       .L_handled
.L_notea:

        @ --- transfers with no width masking, no ZN (TXS, TCS) ---
        cmp     r1, #0x9a               @ TXS: sp = x  (no ZN)
        bne     .L_not9a
.Lop9a:
        ldrh    r2, [r4, #CPU_X]
        strh    r2, [r4, #CPU_SP]
        b       .L_handled
.L_not9a:
        cmp     r1, #0x1b               @ TCS: sp = a  (no ZN, no e-mask here)
        bne     .L_not1b
.Lop1b:
        ldrh    r2, [r4, #CPU_A]
        strh    r2, [r4, #CPU_SP]
        b       .L_handled
.L_not1b:

        @ --- always-16-bit transfers (TCD, TDC, TSC): setZN(value, false) ---
        cmp     r1, #0x5b               @ TCD: dp = a ; ZN(a, word)
        bne     .L_not5b
.Lop5b:
        ldrh    r2, [r4, #CPU_A]
        strh    r2, [r4, #CPU_DP]
        movs    r1, #0                  @ byte-mode = 0
        SETZN
        b       .L_handled
.L_not5b:
        cmp     r1, #0x7b               @ TDC: a = dp ; ZN(dp, word)
        bne     .L_not7b
.Lop7b:
        ldrh    r2, [r4, #CPU_DP]
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled
.L_not7b:
        cmp     r1, #0x3b               @ TSC: a = sp ; ZN(sp, word)
        bne     .L_not3b
.Lop3b:
        ldrh    r2, [r4, #CPU_SP]
        strh    r2, [r4, #CPU_A]
        movs    r1, #0
        SETZN
        b       .L_handled
.L_not3b:

        @ --- accumulator <-> A transfers gated on M flag ---
        @ TXA (0x8a): mf ? a=(a&ff00)|(x&ff) : a=x ; ZN(a, mf)
        cmp     r1, #0x8a
        bne     .L_not8a
.Lop8a:
        LDB     r1, CPU_MF             @ r1 = mf (byte-mode for ZN)
        ldrh    r3, [r4, #CPU_A]        @ r3 = current A (need high byte if mf)
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .L8a_m
        @ word: a = x
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2                  @ already 16-bit clean for ZN
        b       .L8a_zn
.L8a_m: @ byte: a = (a&ff00) | (x&ff)
        and     r3, r3, #0xff00
        uxtb    r2, r2                  @ x & 0xff
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2                  @ ZN on byte
.L8a_zn:
        SETZN
        b       .L_handled
.L_not8a:

        @ TYA (0x98): mf ? a=(a&ff00)|(y&ff) : a=y ; ZN(a, mf)
        cmp     r1, #0x98
        bne     .L_not98
.Lop98:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .L98_m
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        b       .L98_zn
.L98_m:
        and     r3, r3, #0xff00
        uxtb    r2, r2
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L98_zn:
        SETZN
        b       .L_handled
.L_not98:

        @ --- A -> X/Y transfers gated on X flag ---
        @ TAX (0xaa): xf ? x=a&ff : x=a ; ZN(x, xf)
        cmp     r1, #0xaa
        bne     .L_notaa
.Lopaa:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_A]
        cmp     r1, #0
        bne     .Laa_x
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Laa_zn
.Laa_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Laa_zn:
        SETZN
        b       .L_handled
.L_notaa:

        @ TAY (0xa8): xf ? y=a&ff : y=a ; ZN(y, xf)
        cmp     r1, #0xa8
        bne     .L_nota8
.Lopa8:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_A]
        cmp     r1, #0
        bne     .La8_x
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .La8_zn
.La8_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.La8_zn:
        SETZN
        b       .L_handled
.L_nota8:

        @ TSX (0xba): xf ? x=sp&ff : x=sp ; ZN(x, xf)
        cmp     r1, #0xba
        bne     .L_notba
.Lopba:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_SP]
        cmp     r1, #0
        bne     .Lba_x
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Lba_zn
.Lba_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Lba_zn:
        SETZN
        b       .L_handled
.L_notba:

        @ TXY (0x9b): xf ? y=x&ff : y=x ; ZN(y, xf)
        cmp     r1, #0x9b
        bne     .L_not9b
.Lop9b:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .L9b_x
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .L9b_zn
.L9b_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.L9b_zn:
        SETZN
        b       .L_handled
.L_not9b:

        @ TYX (0xbb): xf ? x=y&ff : x=y ; ZN(x, xf)
        cmp     r1, #0xbb
        bne     .L_notbb
.Lopbb:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .Lbb_x
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Lbb_zn
.Lbb_x:
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Lbb_zn:
        SETZN
        b       .L_handled
.L_notbb:

        @ --- accumulator inc/dec gated on M flag ---
        @ INA (0x1a): mf ? a=(a&ff00)|((a+1)&ff) : a++ ; ZN(a, mf)
        cmp     r1, #0x1a
        bne     .L_not1a
.Lop1a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        cmp     r1, #0
        bne     .L1a_m
        adds    r2, r3, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        b       .L1a_zn
.L1a_m:
        @ r3 = A. Keep high byte in r3, compute (low+1)&0xff separately.
        mov     r2, r3                  @ r2 = A
        and     r3, r3, #0xff00         @ r3 = high byte
        uxtb    r2, r2                  @ r2 = low byte
        adds    r2, r2, #1
        uxtb    r2, r2
        orr     r2, r3, r2             @ a = high | new_low
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L1a_zn:
        SETZN
        b       .L_handled
.L_not1a:

        @ DEA (0x3a): mf ? a=(a&ff00)|((a-1)&ff) : a-- ; ZN(a, mf)
        cmp     r1, #0x3a
        bne     .L_not3a
.Lop3a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        cmp     r1, #0
        bne     .L3a_m
        subs    r2, r3, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_A]
        uxth    r2, r2
        b       .L3a_zn
.L3a_m:
        @ r3 = A. Keep high byte in r3, compute (low-1)&0xff separately.
        mov     r2, r3                  @ r2 = A
        and     r3, r3, #0xff00         @ r3 = high byte
        uxtb    r2, r2                  @ r2 = low byte
        subs    r2, r2, #1
        uxtb    r2, r2
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L3a_zn:
        SETZN
        b       .L_handled
.L_not3a:

        @ --- index inc/dec gated on X flag ---
        @ INX (0xe8): xf ? x=(x+1)&ff : x++ ; ZN(x, xf)
        cmp     r1, #0xe8
        bne     .L_note8
.Lope8:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .Le8_x
        adds    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Le8_zn
.Le8_x:
        adds    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Le8_zn:
        SETZN
        b       .L_handled
.L_note8:

        @ DEX (0xca): xf ? x=(x-1)&ff : x-- ; ZN(x, xf)
        cmp     r1, #0xca
        bne     .L_notca
.Lopca:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_X]
        cmp     r1, #0
        bne     .Lca_x
        subs    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_X]
        uxth    r2, r2
        b       .Lca_zn
.Lca_x:
        subs    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_X]
.Lca_zn:
        SETZN
        b       .L_handled
.L_notca:

        @ INY (0xc8): xf ? y=(y+1)&ff : y++ ; ZN(y, xf)
        cmp     r1, #0xc8
        bne     .L_notc8
.Lopc8:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .Lc8_x
        adds    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .Lc8_zn
.Lc8_x:
        adds    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.Lc8_zn:
        SETZN
        b       .L_handled
.L_notc8:

        @ DEY (0x88): xf ? y=(y-1)&ff : y-- ; ZN(y, xf)
        cmp     r1, #0x88
        bne     .L_not88
.Lop88:
        LDB     r1, CPU_XF
        ldrh    r2, [r4, #CPU_Y]
        cmp     r1, #0
        bne     .L88_x
        subs    r2, r2, #1
        uxth    r2, r2
        strh    r2, [r4, #CPU_Y]
        uxth    r2, r2
        b       .L88_zn
.L88_x:
        subs    r2, r2, #1
        uxtb    r2, r2
        strh    r2, [r4, #CPU_Y]
.L88_zn:
        SETZN
        b       .L_handled
.L_not88:

        @ --- XBA (0xeb): swap A bytes ; ZN(new low = old high, byte) ---
        @ C: low=a&ff; high=a>>8; a=(low<<8)|high; setZN(high, true)
        @    i.e. n/z reflect the NEW low byte (which is the old high byte).
        cmp     r1, #0xeb
        bne     .L_noteb
.Lopeb:
        ldrh    r2, [r4, #CPU_A]       @ r2 = old a, e.g. 0x0000AABB
        rev16   r2, r2                 @ swap bytes in halfword: 0x0000BBAA
        uxth    r2, r2                 @ new a = 0xBBAA (old low promoted to high)
        strh    r2, [r4, #CPU_A]
        @ ZN reflects new low byte = old high byte (0xAA here).
        uxtb    r2, r2                 @ r2 = new low byte
        movs    r1, #1                 @ byte mode
        SETZN
        b       .L_handled

        @ --- accumulator shifts (implied form: no operand, no bus access) ---
        @ Gate on M flag exactly as cpu_doOpcode. r1 holds mf (=ZN byte-mode),
        @ r2 the ZN value, r0/r3 scratch. Carry extracted via ubfx (0/1), so the
        @ stored _Bool matches the C compiler's normalisation.
        @ ASL A (0x0a): mf? c=bit7,a=(a&ff00)|((a<<1)&ff) : c=bit15,a=(a<<1)&ffff
.Lop0a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        cmp     r1, #0
        bne     .L0a_m
        ubfx    r2, r3, #15, #1        @ word carry = bit15
        strb    r2, [r4, #CPU_C]
        lsls    r2, r3, #1
        uxth    r2, r2                 @ a = (a<<1)&0xffff
        strh    r2, [r4, #CPU_A]
        b       .L0a_zn
.L0a_m:
        ubfx    r2, r3, #7, #1         @ byte carry = bit7
        strb    r2, [r4, #CPU_C]
        lsls    r0, r3, #1
        uxtb    r0, r0                 @ (a<<1)&0xff
        and     r3, r3, #0xff00
        orr     r2, r3, r0             @ a = (a&ff00)|newlow
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2                 @ ZN byte value
.L0a_zn:
        SETZN
        b       .L_handled

        @ LSR A (0x4a): c=a&1; mf? a=(a&ff00)|((a>>1)&7f) : a=a>>1
.Lop4a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ubfx    r2, r3, #0, #1         @ carry = bit0 (same both widths)
        strb    r2, [r4, #CPU_C]
        cmp     r1, #0
        bne     .L4a_m
        lsrs    r2, r3, #1
        strh    r2, [r4, #CPU_A]
        b       .L4a_zn
.L4a_m:
        uxtb    r0, r3                 @ low byte
        lsrs    r0, r0, #1
        and     r3, r3, #0xff00
        orr     r2, r3, r0
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L4a_zn:
        SETZN
        b       .L_handled

        @ ROL A (0x2a): result=(a<<1)|c; mf? c=bit7,a=(a&ff00)|(result&ff)
        @                                   : c=bit15,a=result&ffff.  bit7/15 of
        @ a == bit8/16 of result (c only sets bit0), so carry comes from a.
.Lop2a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ldrb    r0, [r4, #CPU_C]       @ old carry (shift-in)
        cmp     r1, #0
        bne     .L2a_m
        ubfx    r2, r3, #15, #1        @ word carry = bit15 of a
        strb    r2, [r4, #CPU_C]
        lsls    r2, r3, #1
        orr     r2, r2, r0             @ (a<<1)|c
        uxth    r2, r2                 @ a = result&0xffff
        strh    r2, [r4, #CPU_A]
        b       .L2a_zn
.L2a_m:
        ubfx    r2, r3, #7, #1         @ byte carry = bit7 of a
        strb    r2, [r4, #CPU_C]
        lsls    r2, r3, #1
        orr     r2, r2, r0
        uxtb    r2, r2                 @ result&0xff
        and     r3, r3, #0xff00
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L2a_zn:
        SETZN
        b       .L_handled

        @ ROR A (0x6a): newc=a&1; mf? a=(a&ff00)|((a>>1)&7f)|(c<<7)
        @                                   : a=(a>>1)|(c<<15).  a uses OLD c.
.Lop6a:
        LDB     r1, CPU_MF
        ldrh    r3, [r4, #CPU_A]
        ldrb    r0, [r4, #CPU_C]       @ old carry (shift-in)
        ubfx    r2, r3, #0, #1         @ new carry = bit0
        strb    r2, [r4, #CPU_C]       @ store now; a below uses r0, not CPU_C
        cmp     r1, #0
        bne     .L6a_m
        lsrs    r2, r3, #1
        lsls    r0, r0, #15
        orr     r2, r2, r0            @ a = (a>>1)|(c<<15)
        strh    r2, [r4, #CPU_A]
        b       .L6a_zn
.L6a_m:
        uxtb    r2, r3                 @ low byte
        lsrs    r2, r2, #1
        lsls    r0, r0, #7             @ c<<7
        orr     r2, r2, r0
        and     r3, r3, #0xff00
        orr     r2, r3, r2
        strh    r2, [r4, #CPU_A]
        uxtb    r2, r2
.L6a_zn:
        SETZN
        b       .L_handled

@ --- immediate ALU/load (Stage 3C) ---------------------------------------
@ r6 packing survives each BL snes_cpuRead:
@   bits  7:0  original opcode
@   bit     8  byte mode (MF for A ops, XF for X/Y ops)
@   bits 23:16 low operand byte
@   bits 31:24 high operand byte (word mode only)
.Lop09:                                 @ ORA #imm(m)
.Lop29:                                 @ AND #imm(m)
.Lop49:                                 @ EOR #imm(m)
.Lopa9:                                 @ LDA #imm(m)
        ldrb    r0, [r4, #CPU_MF]
        orr     r6, r6, r0, lsl #8
        b       .Limm_common

.Lopa0:                                 @ LDY #imm(x)
.Lopa2:                                 @ LDX #imm(x)
        ldrb    r0, [r4, #CPU_XF]
        orr     r6, r6, r0, lsl #8

.Limm_common:
        IMM_FETCH_BYTE                  @ low operand
        orr     r6, r6, r0, lsl #16
        tst     r6, #0x100
        bne     .Limm_value_ready
        IMM_FETCH_BYTE                  @ high operand; rebuild K:PC after wrap
        orr     r6, r6, r0, lsl #24
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1              @ 16-bit M/X: one dynamic extra cycle
        strb    r0, [r4, #CPU_CYCLESUSED]

.Limm_value_ready:
        ubfx    r2, r6, #16, #8
        ubfx    r3, r6, #24, #8
        orr     r2, r2, r3, lsl #8      @ operand value (high is zero in byte mode)
        uxtb    r0, r6                   @ original opcode
        ubfx    r1, r6, #8, #1          @ SETZN byte-mode flag

        cmp     r0, #0x29               @ AND is the measured hot ALU case
        beq     .Limm_and
        cmp     r0, #0xa9
        beq     .Limm_lda
        cmp     r0, #0xa0
        beq     .Limm_ldy
        cmp     r0, #0xa2
        beq     .Limm_ldx
        cmp     r0, #0x09
        beq     .Limm_ora
        @ only remaining shared entry is EOR # (0x49)
        ldrh    r3, [r4, #CPU_A]
        eor     r0, r3, r2
        b       .Limm_store_a

.Limm_and:
        ldrh    r3, [r4, #CPU_A]
        and     r0, r3, r2
        b       .Limm_store_a
.Limm_ora:
        ldrh    r3, [r4, #CPU_A]
        orr     r0, r3, r2
        b       .Limm_store_a
.Limm_lda:
        mov     r0, r2

.Limm_store_a:
        cmp     r1, #0
        beq     .Limm_store_a_word
        ldrh    r3, [r4, #CPU_A]
        bfi     r3, r0, #0, #8          @ byte A ops preserve A[15:8]
        strh    r3, [r4, #CPU_A]
        uxtb    r2, r0                  @ SETZN requires width-masked value
        SETZN
        b       .L_handled
.Limm_store_a_word:
        uxth    r2, r0
        strh    r2, [r4, #CPU_A]
        SETZN
        b       .L_handled

.Limm_ldx:
        strh    r2, [r4, #CPU_X]        @ byte mode zero-extends; no high preserve
        SETZN
        b       .L_handled
.Limm_ldy:
        strh    r2, [r4, #CPU_Y]
        SETZN
        b       .L_handled

@ REP/SEP update only mask-selected status fields, then reproduce every
@ cpu_setFlags side effect.  Shifting the mask exposes old bit0 in APSR.C;
@ field order is P = N V M X D I Z C.
.Lopc2:                                 @ REP #imm: selected bits become 0
        movs    r6, #0
        b       .Lflags_imm
.Lope2:                                 @ SEP #imm: selected bits become 1
        movs    r6, #1
.Lflags_imm:
        IMM_FETCH_BYTE
        mov     r2, r6                  @ store value; survives no further BL
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_C]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_Z]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_I]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_D]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_XF]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_MF]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_V]
        lsrs    r0, r0, #1
        it      cs
        strbcs  r2, [r4, #CPU_N]

        ldrb    r0, [r4, #CPU_E]
        cbz     r0, .Lflags_xf
        movs    r0, #1
        strb    r0, [r4, #CPU_MF]       @ emulation mode pins M=X=1
        strb    r0, [r4, #CPU_XF]
        ldrh    r1, [r4, #CPU_SP]
        uxtb    r1, r1
        add.w   r1, r1, #0x100
        strh    r1, [r4, #CPU_SP]
.Lflags_xf:
        ldrb    r0, [r4, #CPU_XF]
        cbz     r0, .L_handled
        ldrh    r1, [r4, #CPU_X]
        uxtb    r1, r1
        strh    r1, [r4, #CPU_X]
        ldrh    r1, [r4, #CPU_Y]
        uxtb    r1, r1
        strh    r1, [r4, #CPU_Y]
        b       .L_handled

@ --- relative branches (Stage 3B): one operand byte at pc, taken-cycle gate ---
@ r6 = 0 (not taken) / 1 (taken, +1 cycle) / 2 (BRA, taken, no cycle).
@ r6 is callee-saved (pushed in prologue), so it survives bl snes_cpuRead.
.Lop10:                                 @ BPL  taken if n==0
        ldrb    r6, [r4, #CPU_N]
        eors    r6, r6, #1
        b       .Lbr_common
.Lop50:                                 @ BVC  taken if v==0
        ldrb    r6, [r4, #CPU_V]
        eors    r6, r6, #1
        b       .Lbr_common
.Lop90:                                 @ BCC  taken if c==0
        ldrb    r6, [r4, #CPU_C]
        eors    r6, r6, #1
        b       .Lbr_common
.Lopd0:                                 @ BNE  taken if z==0
        ldrb    r6, [r4, #CPU_Z]
        eors    r6, r6, #1
        b       .Lbr_common
.Lop30:                                 @ BMI  taken if n==1
        ldrb    r6, [r4, #CPU_N]
        b       .Lbr_common
.Lop70:                                 @ BVS  taken if v==1
        ldrb    r6, [r4, #CPU_V]
        b       .Lbr_common
.Lopb0:                                 @ BCS  taken if c==1
        ldrb    r6, [r4, #CPU_C]
        b       .Lbr_common
.Lopf0:                                 @ BEQ  taken if z==1
        ldrb    r6, [r4, #CPU_Z]
        b       .Lbr_common
.Lop80:                                 @ BRA  always, no taken-cycle
        movs    r6, #2
        b       .Lbr_common

.Lbr_common:
        @ raw operand fetch -- mirrors the step opcode fetch (lines 155-162).
        ldr     r0, [r4, #CPU_MEM]
        ldrb    r1, [r4, #CPU_K]
        ldrh    r2, [r4, #CPU_PC]
        lsl     r1, r1, #16
        orr     r1, r1, r2
        adds    r3, r2, #1
        strh    r3, [r4, #CPU_PC]       @ pc++ (uint16 wrap)
        bl      snes_cpuRead            @ r0 = operand byte
        sxtb    r0, r0                  @ (int8_t)operand
        cbz     r6, .L_handled          @ not taken: operand consumed, done
        ldrh    r2, [r4, #CPU_PC]       @ pc (already +1)
        adds    r2, r2, r0              @ pc += offset (uint16 wrap via strh)
        strh    r2, [r4, #CPU_PC]
        cmp     r6, #2
        beq     .L_handled              @ BRA: no taken-cycle
        ldrb    r0, [r4, #CPU_CYCLESUSED]
        adds    r0, r0, #1
        strb    r0, [r4, #CPU_CYCLESUSED]
        b       .L_handled
.L_noteb:

        @ --- unsupported: try returns 0; step returns fetched opcode ---
        cmp     r5, #0
        ite     eq
        moveq   r0, #0
        movne   r0, r6
        pop     {r4, r5, r6, pc}

.L_handled:
        @ try returns 1; step returns -1.
        cmp     r5, #0
        ite     eq
        moveq   r0, #1
        movne   r0, #-1
        pop     {r4, r5, r6, pc}
        .size   snes_thumb2_try, .-snes_thumb2_try
        .size   snes_thumb2_step, .-snes_thumb2_step

@ Keep the Stage 0 probe: it is still a build-path smoke test (offset header
@ resolves inside assembly; the .S rule + snes_redefines pass produces a Thumb
@ object). Unreferenced under release, so GC'd at link; costs nothing in the
@ firmware image.
        .section .text.snes_thumb2_probe,"ax",%progbits
        .align  2
        .global snes_thumb2_probe
        .type   snes_thumb2_probe,%function
        .thumb_func
snes_thumb2_probe:
        ldrh    r1, [r0, #CPU_PC]
        movs    r0, #0
        bx      lr
        .size   snes_thumb2_probe, .-snes_thumb2_probe

        .section        .note.GNU-stack,"",%progbits
