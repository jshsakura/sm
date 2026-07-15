
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


#include "apu.h"
#include "snes.h"
#include "spc.h"
#include "dsp.h"
#include "../tracing.h"

static const uint8_t bootRom[0x40] = {
  0xcd, 0xef, 0xbd, 0xe8, 0x00, 0xc6, 0x1d, 0xd0, 0xfc, 0x8f, 0xaa, 0xf4, 0x8f, 0xbb, 0xf5, 0x78,
  0xcc, 0xf4, 0xd0, 0xfb, 0x2f, 0x19, 0xeb, 0xf4, 0xd0, 0xfc, 0x7e, 0xf4, 0xd0, 0x0b, 0xe4, 0xf5,
  0xcb, 0xf4, 0xd7, 0x00, 0xfc, 0xd0, 0xf3, 0xab, 0x01, 0x10, 0xef, 0x7e, 0xf4, 0x10, 0xeb, 0xba,
  0xf6, 0xda, 0x00, 0xba, 0xf4, 0xc4, 0xf4, 0xdd, 0x5d, 0xd0, 0xdb, 0x1f, 0x00, 0x00, 0xc0, 0xff
};

Apu* apu_init(void) {
#ifdef TARGET_GNW
  /* 66 KB (64 KB of it ARAM). The firmware's main heap is 85 KB and shared, so
   * take it from AHB SRAM, which nothing else in this core touches. */
  extern void *ahb_malloc(size_t size);
  Apu* apu = ahb_malloc(sizeof(Apu));
#else
  Apu* apu = malloc(sizeof(Apu));
#endif
  apu->spc = spc_init(apu);
  apu->dsp = dsp_init(apu->ram);
  return apu;
}

void apu_free(Apu* apu) {
  spc_free(apu->spc);
  dsp_free(apu->dsp);
  free(apu);
}

void apu_reset(Apu* apu) {
  apu->romReadable = true; // before resetting spc, because it reads reset vector from it
  spc_reset(apu->spc);
  dsp_reset(apu->dsp);
  memset(apu->ram, 0, sizeof(apu->ram));
  apu->dspAdr = 0;
  apu->cycles = 0;
  memset(apu->inPorts, 0, sizeof(apu->inPorts));
  memset(apu->outPorts, 0, sizeof(apu->outPorts));
  for(int i = 0; i < 3; i++) {
    apu->timer[i].cycles = 0;
    apu->timer[i].divider = 0;
    apu->timer[i].target = 0;
    apu->timer[i].counter = 0;
    apu->timer[i].enabled = false;
  }
  apu->cpuCyclesLeft = 7;
  apu->hist.count = 0;
}

void apu_saveload(Apu *apu, SaveLoadFunc *func, void *ctx) {
  func(ctx, apu->ram, offsetof(Apu, pad) + 6 - offsetof(Apu, ram));
  dsp_saveload(apu->dsp, func, ctx);
  spc_saveload(apu->spc, func, ctx);
}

bool g_debug_apu_cycles;

void apu_cycle(Apu* apu) {
  if(apu->cpuCyclesLeft == 0) {
    if (g_debug_apu_cycles) {
      char line[80];
      getProcessorStateSpc(apu, line);
      puts(line);
    }
    apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
  }
  apu->cpuCyclesLeft--;

  if((apu->cycles & 0x1f) == 0) {
    // every 32 cycles
    dsp_cycle(apu->dsp);
  }

  // handle timers
  for(int i = 0; i < 3; i++) {
    if(apu->timer[i].cycles == 0) {
      apu->timer[i].cycles = i == 2 ? 16 : 128;
      if(apu->timer[i].enabled) {
        apu->timer[i].divider++;
        if(apu->timer[i].divider == apu->timer[i].target) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter++;
          apu->timer[i].counter &= 0xf;
        }
      }
    }
    apu->timer[i].cycles--;
  }

  apu->cycles++;
}

/* Advance the APU by `cyclesToRun` SPC cycles. Identical machine to calling
 * apu_cycle() that many times, but the per-cycle DSP tick and the three timers are
 * folded into closed-form bulk updates between opcode boundaries — the SPC700's idle
 * cycles charged in one step, exactly as the main CPU's dot loop was collapsed
 * (snes_run_line). apu_cycle() called this ~17,000x/frame doing a 3-timer loop and a
 * DSP branch every single cycle; an opcode already told us its whole cost. Cycle-exact:
 * the framebuffer/WRAM/SRAM state hash is bit-identical to the per-cycle loop. */
void apu_run(Apu* apu, int cyclesToRun) {
  while (cyclesToRun > 0) {
    if (apu->cpuCyclesLeft == 0)
      apu->cpuCyclesLeft = spc_runOpcode(apu->spc);

    int step = apu->cpuCyclesLeft < cyclesToRun ? apu->cpuCyclesLeft : cyclesToRun;
    if (step <= 0) step = 1;   /* an opcode charging 0: step one and wrap like the ref */

    /* DSP fires when (cycles & 0x1f)==0, tested before the increment — so once for
     * every multiple of 32 in [cycles, cycles+step). */
    uint32_t start = apu->cycles, end = start + (uint32_t)step;
    for (uint32_t m = (start + 31u) & ~31u; m < end; m += 32u)
      dsp_cycle(apu->dsp);

    /* Each timer counts down; when it passes 0 it reloads to R and, if enabled,
     * advances divider->counter. Over `step` cycles the zero-crossings land at
     * k = C, C+R, C+2R, ... for k in [0,step). step <= cpuCyclesLeft (<=255), so the
     * fallback loops are tiny. */
    for (int i = 0; i < 3; i++) {
      Timer* t = &apu->timer[i];
      int R = (i == 2) ? 16 : 128;
      int C = t->cycles;
      int ticks = (C < step) ? ((step - 1 - C) / R + 1) : 0;
      if (ticks) {
        if (t->enabled) {
          if (t->target && t->divider < t->target) {
            int total = t->divider + ticks;
            t->counter = (uint8_t)((t->counter + total / t->target) & 0xf);
            t->divider = (uint8_t)(total % t->target);
          } else {
            /* target 0 (==test only on the 256-wrap) or divider>=target: step it */
            for (int k = 0; k < ticks; k++) {
              t->divider++;
              if (t->divider == t->target) { t->divider = 0; t->counter = (t->counter + 1) & 0xf; }
            }
          }
        }
        int lastK = C + (ticks - 1) * R;
        t->cycles = (uint8_t)(R + lastK - step);   /* value after the last reload */
      } else {
        t->cycles = (uint8_t)(C - step);
      }
    }

    apu->cycles = end;
    apu->cpuCyclesLeft -= (uint8_t)step;
    cyclesToRun -= step;
  }
}

uint8_t apu_cpuRead(Apu* apu, uint16_t adr) {
  switch(adr) {
    case 0xf0:
    case 0xf1:
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      return 0;
    }
    case 0xf2: {
      return apu->dspAdr;
    }
    case 0xf3: {
      return dsp_read(apu->dsp, apu->dspAdr & 0x7f);
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7:
    case 0xf8:
    case 0xf9: {
      return apu->inPorts[adr - 0xf4];
    }
    case 0xfd:
    case 0xfe:
    case 0xff: {
      uint8_t ret = apu->timer[adr - 0xfd].counter;
      apu->timer[adr - 0xfd].counter = 0;
      return ret;
    }
  }
  if(apu->romReadable && adr >= 0xffc0) {
    return bootRom[adr - 0xffc0];
  }
  return apu->ram[adr];
}

void apu_cpuWrite(Apu* apu, uint16_t adr, uint8_t val) {
  switch(adr) {
    case 0xf0: {
      break; // test register
    }
    case 0xf1: {
      for(int i = 0; i < 3; i++) {
        if(!apu->timer[i].enabled && (val & (1 << i))) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter = 0;
        }
        apu->timer[i].enabled = val & (1 << i);
      }
      if(val & 0x10) {
        apu->inPorts[0] = 0;
        apu->inPorts[1] = 0;
      }
      if(val & 0x20) {
        apu->inPorts[2] = 0;
        apu->inPorts[3] = 0;
      }
      apu->romReadable = val & 0x80;
      break;
    }
    case 0xf2: {
      apu->dspAdr = val;
      break;
    }
    case 0xf3: {
      int i = apu->hist.count;
      if (i != 256) {
        apu->hist.count = i + 1;
        apu->hist.addr[i] = (uint8_t)apu->dspAdr;
        apu->hist.val[i] = val;
      }
      if(apu->dspAdr < 0x80) dsp_write(apu->dsp, apu->dspAdr, val);
      break;
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7: {
      apu->outPorts[adr - 0xf4] = val;
      break;
    }
    case 0xf8:
    case 0xf9: {
      apu->inPorts[adr - 0xf4] = val;
      break;
    }
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      apu->timer[adr - 0xfa].target = val;
      break;
    }
  }
  apu->ram[adr] = val;
}
