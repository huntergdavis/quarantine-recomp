// CPU-level runtime: state, dispatch, far control transfer, IRQ delivery, string ops, mul/div, x87.
#include "compat.h"
#include "rt.h"
#include "sys.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#ifndef _WIN32
#include <execinfo.h>
#endif
#include <setjmp.h>
#include <time.h>

uint32_t R[8];
uint16_t S[6];
uint32_t SB[6];
uint32_t DF, IF = 1;
uint8_t *MEM;
uint32_t fl_op = F_LOGIC, fl_sz = 32, fl_a, fl_b, fl_res = 1, fl_c, fl_expl;
double FPR[8];
uint32_t FPU_TOP, FPU_CW = 0x37f, FPU_CC;
volatile int irq_pending;
int rt_verbose;
int rt_skip_copyprot = 1;
// Called at the head of known polling loops (IDLE in recomp.py): give the host CPU back for a moment.
// The timer thread raises irq_pending every 1 ms, so interrupts keep arriving at the same rate.
void rt_idle(void) {
    host_sleep_ns(500000);
    IRQCHK();
}
uint32_t SH[65536]; int SHP = 1;
void rt_ret_jump(uint32_t ra) {
    if (ra == MAGIC_IRQRET || ra == MAGIC_FARRET) rt_fatal("near ret to magic %08x", ra);
    rt_dispatch_tail_noret(ra);
}

_Noreturn void rt_fatal(const char *fmt, ...) {
    char msg[1024];
    va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof msg, fmt, ap); va_end(ap);
    fprintf(stderr, "FATAL: %s\n", msg);
    rt_dump_state(stderr);
    sys_fatal_box(msg);
#ifndef _WIN32
    void *bt[48]; int n = backtrace(bt, 48);
    char **sym = backtrace_symbols(bt, n);
    fprintf(stderr, "host backtrace:");
    for (int k = 0; k < n; k++) { char *f = strstr(sym[k], " f_"); if (!f) f = strstr(sym[k], " r_"); if (!f) f = strstr(sym[k], " rt_");
                                  if (f) { char nm[64]; sscanf(f + 1, "%63s", nm); fprintf(stderr, " %s", nm); } }
    fprintf(stderr, "\n");
#endif
    sys_shutdown();
    exit(1);
}

void rt_unimpl(uint32_t addr, const char *what) {
    rt_fatal("unimplemented instruction at %08x (guest %08x): %s", addr, addr - 0x100000, what);
}

void rt_dump_state(FILE *f) {
    fprintf(f, "eax=%08x ecx=%08x edx=%08x ebx=%08x esp=%08x ebp=%08x esi=%08x edi=%08x\n",
            EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI);
    fprintf(f, "es=%04x cs=%04x ss=%04x ds=%04x fs=%04x gs=%04x  flags=%03x\n",
            S[0], S[1], S[2], S[3], S[4], S[5], get_eflags());
    fprintf(f, "stack:");
    for (int k = 0; k < 12; k++) fprintf(f, " %08x", RD32(ESP + 4 * k));
    fprintf(f, "\n");
    rt_trace_dump(f);
}

// ---------------------------------------------------------------- call trace ring (debug)
#define TRACE_N 256
static uint32_t trace_ring[TRACE_N]; static int trace_pos;
void rt_trace(uint32_t a) { trace_ring[trace_pos++ & (TRACE_N - 1)] = a; }
#ifdef SMC_TRACE
// debug: log each distinct code address the game writes to, with the function that was running
void rt_smc_note(uint32_t a, uint32_t v, int sz) {
    static uint8_t seen[0x60000];
    uint32_t o = a - 0x110000u;
    if (seen[o]) return; seen[o] = 1;
    uint32_t last = trace_ring[(trace_pos - 1) & (TRACE_N - 1)];
    int in_region = 0; for (int k = 0; k < NREGIONS; k++) if (a >= REGIONS[k].lo && a < REGIONS[k].hi) in_region = 1;
    fprintf(stderr, "[smc] write%d %08x (guest %08x) = %08x, last dispatch %08x%s\n", sz, a, a - 0x100000, v, last, in_region ? " [handled region]" : "");
}
#endif
void rt_trace_dump(FILE *f) {
    fprintf(f, "last dispatches:");
    for (int k = 0; k < TRACE_N; k++) {
        uint32_t a = trace_ring[(trace_pos + k) & (TRACE_N - 1)];
        if (a) fprintf(f, " %x", a);
    }
    fprintf(f, "\n");
}

#ifdef RT_DEBUG
void rt_fn_enter(uint32_t a) {
    rt_trace(a);
    if (ESP < 0x180000 || ESP > MEM_SIZE) rt_fatal("bad ESP %08x entering %08x", ESP, a);
}
#endif
// ---------------------------------------------------------------- dispatch
static gfn lookup(uint32_t a) {
    int lo = 0, hi = NFUNCS - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (FUNCS[mid].addr == a) return FUNCS[mid].fn;
        if (FUNCS[mid].addr < a) lo = mid + 1; else hi = mid - 1;
    }
    return 0;
}

static inline void esp_check(uint32_t a) {
    if (ESP < 0x180000 || ESP > MEM_SIZE) rt_fatal("bad ESP %08x on dispatch to %08x", ESP, a);
}
void rt_dispatch(uint32_t a) {
    rt_trace(a); esp_check(a);
    gfn f = lookup(a);
    if (f) { f(); return; }
    for (int k = 0; k < NREGIONS; k++)
        if (a >= REGIONS[k].lo && a < REGIONS[k].hi) { REGIONS[k].fn(a); return; }
    if (a >= MAGIC_BASE) { sys_magic(a); return; }
    rt_fatal("dispatch to unknown address %08x (guest %08x)", a, a - 0x100000);
}
void rt_dispatch_tail(uint32_t a) { rt_dispatch(a); }

// far returns: lcall pushes MAGIC_FARRET; anything else is a retf used as a jump
void rt_retf(int n) {
    uint32_t off = POP32(); POP32(); ESP += n;
    if (rt_verbose) fprintf(stderr, "retf -> %08x esp=%08x\n", off, ESP);
    if (off == MAGIC_FARRET) return;
    rt_dispatch_tail_noret(off);      // retf used as a jump (e.g. _chain_intr)
}
void rt_dispatch_tail_noret(uint32_t off) {
    // A jump with no return address pushed: the callee consumes whatever is on the stack.
    rt_trace(off | 0x80000000u); esp_check(off);
    gfn f = lookup(off);
    if (f) { f(); return; }
    if (off >= MAGIC_BASE) { sys_magic(off); return; }
    for (int k = 0; k < NREGIONS; k++)
        if (off >= REGIONS[k].lo && off < REGIONS[k].hi) { REGIONS[k].fn(off); return; }
    rt_fatal("far jump to unknown %08x", off);
}
// Interrupt delivery: an iret that returns to MAGIC_IRQRET ends the interrupt, however deep the
// C stack is (handlers may abandon their frames, e.g. Watcom _chain_intr), so unwind with longjmp.
static jmp_buf irq_jb[8]; static int irq_shp[8]; static volatile int irq_depth;
void rt_iret(void) {
    uint32_t eip = POP32(); POP32(); set_eflags(POP32());
    if (eip == MAGIC_IRQRET && irq_depth > 0) { irq_depth--; longjmp(irq_jb[irq_depth], 1); }
}
void rt_deliver_int(uint32_t handler) {
    PUSH32(get_eflags()); PUSH32(S[1]); PUSH32(MAGIC_IRQRET);
    if (irq_depth >= 8) rt_fatal("interrupt nesting too deep");
    int d = irq_depth;
    irq_shp[d] = SHP;
    if (setjmp(irq_jb[d]) == 0) {
        irq_depth = d + 1;
        rt_dispatch_tail_noret(handler);
        rt_fatal("interrupt handler %08x returned without iret", handler);
    }
    SHP = irq_shp[d];
}
void rt_farjmp(uint32_t off, uint16_t sel) { (void)sel; rt_dispatch_tail_noret(off); }
void rt_farcall(uint32_t off, uint16_t sel, uint32_t ret) {
    (void)ret;
    static int logged; if (rt_verbose && logged++ < 40) fprintf(stderr, "farcall %04x:%08x esp=%08x\n", sel, off, ESP);
    if (sel == 2) { sys_obj2_call(off); return; }   // 16-bit joystick helper object
    PUSH32(S[1]); PUSH32(MAGIC_FARRET);
    rt_dispatch(off);
}

// ---------------------------------------------------------------- IRQ delivery
static int in_irq;
void rt_irq(void) {
    irq_pending = 0;
    sys_host_tick();
    if (in_irq || !IF) { if (sys_irq_waiting()) irq_pending = 1; return; }
    int vec;
    while ((vec = sys_next_irq()) >= 0) {
        in_irq++;
        uint32_t handler = sys_pm_vector(vec);
        if (rt_verbose) fprintf(stderr, "IRQ vec %d -> %08x esp=%08x\n", vec, handler, ESP);
        IF = 0;
        rt_deliver_int(handler);
        IF = 1;
        in_irq--;
        if (rt_verbose) fprintf(stderr, "IRQ done esp=%08x\n", ESP);
    }
    in_irq++; snd_service(); in_irq--;
}

// ---------------------------------------------------------------- mul / div
void rt_imul1(uint32_t v, int sz) {
    if (sz == 8) { int16_t p = (int8_t)EAX * (int8_t)v; EAX = (EAX & 0xffff0000u) | (uint16_t)p; SETFL(F_MUL, 8, 0, 0, p); fl_c = p != (int8_t)p; }
    else if (sz == 16) { int32_t p = (int16_t)EAX * (int16_t)v; EAX = (EAX & 0xffff0000u) | (uint16_t)p; EDX = (EDX & 0xffff0000u) | (uint16_t)(p >> 16); SETFL(F_MUL, 16, 0, 0, p); fl_c = p != (int16_t)p; }
    else { int64_t p = (int64_t)(int32_t)EAX * (int32_t)v; EAX = (uint32_t)p; EDX = (uint32_t)(p >> 32); SETFL(F_MUL, 32, 0, 0, (uint32_t)p); fl_c = p != (int32_t)p; }
}
void rt_mul1(uint32_t v, int sz) {
    if (sz == 8) { uint16_t p = (uint8_t)EAX * (uint8_t)v; EAX = (EAX & 0xffff0000u) | p; SETFL(F_MUL, 8, 0, 0, p); fl_c = p > 0xff; }
    else if (sz == 16) { uint32_t p = (uint16_t)EAX * (uint16_t)v; EAX = (EAX & 0xffff0000u) | (uint16_t)p; EDX = (EDX & 0xffff0000u) | (p >> 16); SETFL(F_MUL, 16, 0, 0, p); fl_c = p > 0xffff; }
    else { uint64_t p = (uint64_t)EAX * v; EAX = (uint32_t)p; EDX = (uint32_t)(p >> 32); SETFL(F_MUL, 32, 0, 0, (uint32_t)p); fl_c = (p >> 32) != 0; }
}
void rt_div1(uint32_t v, int sz) {
    if (sz == 8) { v &= 0xff; if (!v) rt_fatal("divide by zero"); uint16_t n = (uint16_t)EAX; uint32_t q = n / v; if (q > 0xff) rt_fatal("div overflow"); EAX = (EAX & 0xffff0000u) | ((n % v) << 8) | q; }
    else if (sz == 16) { v &= 0xffff; if (!v) rt_fatal("divide by zero"); uint32_t n = ((EDX & 0xffff) << 16) | (EAX & 0xffff); uint32_t q = n / v; if (q > 0xffff) rt_fatal("div overflow"); EAX = (EAX & 0xffff0000u) | q; EDX = (EDX & 0xffff0000u) | (n % v); }
    else { if (!v) rt_fatal("divide by zero"); uint64_t n = ((uint64_t)EDX << 32) | EAX; uint64_t q = n / v; if (q > 0xffffffffu) rt_fatal("div overflow"); EAX = (uint32_t)q; EDX = (uint32_t)(n % v); }
}
void rt_idiv1(uint32_t v, int sz) {
    if (sz == 8) { int32_t d = (int8_t)v; if (!d) rt_fatal("idiv by zero"); int32_t n = (int16_t)EAX; int32_t q = n / d; if (q > 127 || q < -128) rt_fatal("idiv overflow"); EAX = (EAX & 0xffff0000u) | ((uint8_t)(n % d) << 8) | (uint8_t)q; }
    else if (sz == 16) { int32_t d = (int16_t)v; if (!d) rt_fatal("idiv by zero"); int32_t n = (int32_t)(((EDX & 0xffff) << 16) | (EAX & 0xffff)); int32_t q = n / d; if (q > 32767 || q < -32768) rt_fatal("idiv overflow"); EAX = (EAX & 0xffff0000u) | (uint16_t)q; EDX = (EDX & 0xffff0000u) | (uint16_t)(n % d); }
    else { int64_t d = (int32_t)v; if (!d) rt_fatal("idiv by zero at esp=%08x", ESP); int64_t n = (int64_t)(((uint64_t)EDX << 32) | EAX); int64_t q = n / d; if (q > 2147483647LL || q < -2147483648LL) rt_fatal("idiv overflow %llx / %llx", (long long)n, (long long)d); EAX = (uint32_t)q; EDX = (uint32_t)(n % d); }
}

// ---------------------------------------------------------------- string ops
#define STEP(sz) (DF ? -(int)((sz) / 8) : (int)((sz) / 8))
static inline uint32_t rdsz(uint32_t a, int sz) { return sz == 8 ? RD8(a) : sz == 16 ? RD16(a) : RD32(a); }
static inline void wrsz(uint32_t a, uint32_t v, int sz) { if (sz == 8) WR8(a, v); else if (sz == 16) WR16(a, v); else WR32(a, v); }
static inline void setacc(uint32_t v, int sz) {
    if (sz == 8) EAX = (EAX & ~0xffu) | (v & 0xff); else if (sz == 16) EAX = (EAX & ~0xffffu) | (v & 0xffff); else EAX = v;
}
void rt_str_movs(int sz, int seg, char rep) {
    int st = STEP(sz);
    if (rep == '-') { wrsz(SB[0] + EDI, rdsz(SB[seg] + ESI, sz), sz); ESI += st; EDI += st; return; }
    if (sz == 8 && !DF && !IS_VGA(SB[0] + EDI) && !IS_VGA(SB[seg] + ESI) && ECX < MEM_SIZE &&
        (SB[0] + EDI + ECX) < MEM_SIZE && (SB[seg] + ESI + ECX) < MEM_SIZE &&
        !(SB[0] + EDI > SB[seg] + ESI && SB[0] + EDI < SB[seg] + ESI + ECX)) {
        memmove(MEM + SB[0] + EDI, MEM + SB[seg] + ESI, ECX); ESI += ECX; EDI += ECX; ECX = 0; return;
    }
    while (ECX) { wrsz(SB[0] + EDI, rdsz(SB[seg] + ESI, sz), sz); ESI += st; EDI += st; ECX--; }
}
void rt_str_stos(int sz, int seg, char rep) {
    (void)seg; int st = STEP(sz);
    if (rep == '-') { wrsz(SB[0] + EDI, EAX, sz); EDI += st; return; }
    while (ECX) { wrsz(SB[0] + EDI, EAX, sz); EDI += st; ECX--; }
}
void rt_str_lods(int sz, int seg, char rep) {
    int st = STEP(sz);
    if (rep == '-') { setacc(rdsz(SB[seg] + ESI, sz), sz); ESI += st; return; }
    while (ECX) { setacc(rdsz(SB[seg] + ESI, sz), sz); ESI += st; ECX--; }
}
void rt_str_scas(int sz, int seg, char rep) {
    (void)seg; int st = STEP(sz);
    if (rep == '-') { alu_sub(EAX, rdsz(SB[0] + EDI, sz), sz); EDI += st; return; }
    while (ECX) {
        alu_sub(EAX, rdsz(SB[0] + EDI, sz), sz); EDI += st; ECX--;
        if (rep == 'e' && !get_zf()) break;
        if (rep == 'n' && get_zf()) break;
    }
}
void rt_str_cmps(int sz, int seg, char rep) {
    int st = STEP(sz);
    if (rep == '-') { alu_sub(rdsz(SB[seg] + ESI, sz), rdsz(SB[0] + EDI, sz), sz); ESI += st; EDI += st; return; }
    while (ECX) {
        alu_sub(rdsz(SB[seg] + ESI, sz), rdsz(SB[0] + EDI, sz), sz); ESI += st; EDI += st; ECX--;
        if (rep == 'e' && !get_zf()) break;
        if (rep == 'n' && get_zf()) break;
    }
}
void rt_str_ins(int sz, int seg, char rep) {
    (void)seg; int st = STEP(sz);
    uint32_t p = EDX & 0xffff;
    do { uint32_t v = sz == 8 ? rt_in8(p) : sz == 16 ? rt_in16(p) : rt_in32(p); wrsz(SB[0] + EDI, v, sz); EDI += st; } while (rep != '-' && --ECX);
}
void rt_str_outs(int sz, int seg, char rep) {
    int st = STEP(sz);
    uint32_t p = EDX & 0xffff;
    do { uint32_t v = rdsz(SB[seg] + ESI, sz); if (sz == 8) rt_out8(p, v); else if (sz == 16) rt_out16(p, v); else rt_out32(p, v); ESI += st; } while (rep != '-' && --ECX);
}

// ---------------------------------------------------------------- x87 extras
double fpu_round(double v) {
    switch ((FPU_CW >> 10) & 3) {
    case 0: return nearbyint(v);
    case 1: return floor(v);
    case 2: return ceil(v);
    default: return trunc(v);
    }
}
double fpu_ld_f80(uint32_t a) {
    uint64_t mant = RD32(a) | ((uint64_t)RD32(a + 4) << 32);
    uint16_t se = RD16(a + 8);
    int exp = se & 0x7fff;
    if (!exp && !mant) return (se & 0x8000) ? -0.0 : 0.0;
    double v = ldexp((double)mant, exp - 16383 - 63);
    return (se & 0x8000) ? -v : v;
}
void fpu_st_f80(uint32_t a, double v) {
    uint16_t se = 0; uint64_t mant = 0;
    if (v != 0) {
        if (v < 0) { se = 0x8000; v = -v; }
        int e; double m = frexp(v, &e);          // v = m * 2^e, m in [0.5,1)
        mant = (uint64_t)ldexp(m, 64);
        se |= (uint16_t)(e - 1 + 16383);
    }
    WR32(a, (uint32_t)mant); WR32(a + 4, (uint32_t)(mant >> 32)); WR16(a + 8, se);
}
void fpu_init(void) { FPU_TOP = 0; FPU_CW = 0x37f; FPU_CC = 0; }
void fpu_save(uint32_t a) {
    WR16(a, FPU_CW); WR16(a + 4, fpu_sw()); WR16(a + 8, 0);
    for (int k = 0; k < 8; k++) fpu_st_f80(a + 28 + 10 * k, ST(k));
    fpu_init();
}
void fpu_restore(uint32_t a) {
    FPU_CW = RD16(a); uint16_t sw = RD16(a + 4); FPU_CC = sw & 0x4700; FPU_TOP = (sw >> 11) & 7;
    for (int k = 0; k < 8; k++) ST(k) = fpu_ld_f80(a + 28 + 10 * k);
}
