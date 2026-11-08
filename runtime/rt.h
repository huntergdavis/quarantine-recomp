// Runtime core for recompiled Quarantine: CPU state, lazy flags, memory, ALU helpers.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

// ---------------------------------------------------------------- state
extern uint32_t R[8];            // eax ecx edx ebx esp ebp esi edi
#define EAX R[0]
#define ECX R[1]
#define EDX R[2]
#define EBX R[3]
#define ESP R[4]
#define EBP R[5]
#define ESI R[6]
#define EDI R[7]
extern uint16_t S[6];            // es cs ss ds fs gs
extern uint32_t SB[6];           // segment bases
extern uint32_t DF, IF;
extern uint8_t *MEM;
#define MEM_SIZE (64u << 20)
#define MEM_MASK (MEM_SIZE - 1)

void set_sreg(int s, uint16_t sel);
_Noreturn void rt_fatal(const char *fmt, ...);
void guest_load_exe(const char *dir);   // rt_le.c
void rt_unimpl(uint32_t addr, const char *what);

// ---------------------------------------------------------------- memory
uint8_t vga_read(uint32_t a);
void vga_write(uint32_t a, uint8_t v);
#define IS_VGA(a) ((uint32_t)((a) - 0xA0000u) < 0x10000u)

static inline uint8_t RD8(uint32_t a) {
    if (__builtin_expect(IS_VGA(a), 0)) return vga_read(a);
    return MEM[a & MEM_MASK];
}
static inline uint16_t RD16(uint32_t a) {
    if (__builtin_expect(IS_VGA(a), 0)) return vga_read(a) | (vga_read(a + 1) << 8);
    uint16_t v; memcpy(&v, MEM + (a & MEM_MASK), 2); return v;
}
static inline uint32_t RD32(uint32_t a) {
    if (__builtin_expect(IS_VGA(a), 0)) return RD16(a) | ((uint32_t)RD16(a + 2) << 16);
    uint32_t v; memcpy(&v, MEM + (a & MEM_MASK), 4); return v;
}
#ifdef SMC_TRACE
void rt_smc_note(uint32_t a, uint32_t v, int sz);
#define SMC_NOTE(a, v, sz) do { if ((uint32_t)((a) - 0x110000u) < 0x60000u) rt_smc_note(a, v, sz); } while (0)
#else
#define SMC_NOTE(a, v, sz) ((void)0)
#endif
static inline void WR8(uint32_t a, uint32_t v) {
    SMC_NOTE(a, v, 8);
    if (__builtin_expect(IS_VGA(a), 0)) { vga_write(a, (uint8_t)v); return; }
    MEM[a & MEM_MASK] = (uint8_t)v;
}
static inline void WR16(uint32_t a, uint32_t v) {
    SMC_NOTE(a, v, 16);
    if (__builtin_expect(IS_VGA(a), 0)) { vga_write(a, (uint8_t)v); vga_write(a + 1, (uint8_t)(v >> 8)); return; }
    uint16_t t = (uint16_t)v; memcpy(MEM + (a & MEM_MASK), &t, 2);
}
static inline void WR32(uint32_t a, uint32_t v) {
    SMC_NOTE(a, v, 32);
    if (__builtin_expect(IS_VGA(a), 0)) { WR16(a, v); WR16(a + 2, v >> 16); return; }
    memcpy(MEM + (a & MEM_MASK), &v, 4);
}
static inline void PUSH32(uint32_t v) { ESP -= 4; WR32(SB[2] + ESP, v); }
static inline void PUSH16(uint32_t v) { ESP -= 2; WR16(SB[2] + ESP, v); }
static inline uint32_t POP32(void) { uint32_t v = RD32(SB[2] + ESP); ESP += 4; return v; }
static inline uint32_t POP16(void) { uint32_t v = RD16(SB[2] + ESP); ESP += 2; return v; }

// ---------------------------------------------------------------- lazy flags
enum { F_ADD, F_ADC, F_SUB, F_SBB, F_LOGIC, F_INC, F_DEC, F_NEG, F_SHL, F_SHR, F_SAR, F_EXPL, F_RESCF, F_MUL };
extern uint32_t fl_op, fl_sz, fl_a, fl_b, fl_res, fl_c, fl_expl;
#define FL_CF 0x001u
#define FL_PF 0x004u
#define FL_ZF 0x040u
#define FL_SF 0x080u
#define FL_OF 0x800u

static inline uint32_t MASKSZ(int sz) { return sz == 8 ? 0xffu : sz == 16 ? 0xffffu : 0xffffffffu; }
static inline uint32_t SIGNSZ(int sz) { return sz == 8 ? 0x80u : sz == 16 ? 0x8000u : 0x80000000u; }
static inline int32_t SXT(uint32_t v, int sz) { return sz == 8 ? (int8_t)v : sz == 16 ? (int16_t)v : (int32_t)v; }
#define SETFL(op, sz, a, b, r) (fl_op = (op), fl_sz = (sz), fl_a = (a), fl_b = (b), fl_res = (r))

static inline uint32_t get_cf(void) {
    uint32_t m = MASKSZ(fl_sz);
    switch (fl_op) {
    case F_ADD: return (fl_res & m) < (fl_a & m);
    case F_ADC: return fl_c ? (fl_res & m) <= (fl_a & m) : (fl_res & m) < (fl_a & m);
    case F_SUB: return (fl_a & m) < (fl_b & m);
    case F_SBB: return (uint64_t)(fl_a & m) < (uint64_t)(fl_b & m) + fl_c;
    case F_LOGIC: return 0;
    case F_INC: case F_DEC: case F_RESCF: case F_SHL: case F_SHR: case F_SAR: case F_MUL: return fl_c;
    case F_NEG: return (fl_a & m) != 0;
    default: return fl_expl & FL_CF ? 1 : 0;
    }
}
static inline uint32_t get_zf(void) {
    if (fl_op == F_EXPL) return (fl_expl & FL_ZF) ? 1 : 0;
    return (fl_res & MASKSZ(fl_sz)) == 0;
}
static inline uint32_t get_sf(void) {
    if (fl_op == F_EXPL) return (fl_expl & FL_SF) ? 1 : 0;
    return (fl_res & SIGNSZ(fl_sz)) ? 1 : 0;
}
static inline uint32_t get_of(void) {
    uint32_t s = SIGNSZ(fl_sz);
    switch (fl_op) {
    case F_ADD: case F_ADC: return (((fl_a ^ fl_res) & (fl_b ^ fl_res)) & s) ? 1 : 0;
    case F_SUB: case F_SBB: return (((fl_a ^ fl_b) & (fl_a ^ fl_res)) & s) ? 1 : 0;
    case F_LOGIC: case F_SAR: return 0;
    case F_INC: return (fl_res & MASKSZ(fl_sz)) == s;
    case F_DEC: return (fl_res & MASKSZ(fl_sz)) == s - 1;
    case F_NEG: return (fl_a & MASKSZ(fl_sz)) == s;
    case F_SHL: return ((fl_res & s) ? 1 : 0) ^ fl_c;
    case F_SHR: return (fl_a & s) ? 1 : 0;
    case F_RESCF: return fl_b;
    case F_MUL: return fl_c;
    default: return fl_expl & FL_OF ? 1 : 0;
    }
}
static inline uint32_t get_pf(void) {
    if (fl_op == F_EXPL) return (fl_expl & FL_PF) ? 1 : 0;
    return !__builtin_parity(fl_res & 0xff);
}
static inline int cond_b(void) {
    if (fl_op == F_SUB) { uint32_t m = MASKSZ(fl_sz); return (fl_a & m) < (fl_b & m); }
    return get_cf();
}
static inline int cond_be(void) {
    if (fl_op == F_SUB) { uint32_t m = MASKSZ(fl_sz); return (fl_a & m) <= (fl_b & m); }
    return get_cf() || get_zf();
}
static inline int cond_l(void) {
    if (fl_op == F_SUB) return SXT(fl_a, fl_sz) < SXT(fl_b, fl_sz);
    return get_sf() != get_of();
}
static inline int cond_le(void) {
    if (fl_op == F_SUB) return SXT(fl_a, fl_sz) <= SXT(fl_b, fl_sz);
    return get_zf() || (get_sf() != get_of());
}
static inline uint32_t get_eflags(void) {
    return 0x202u | (get_cf() ? FL_CF : 0) | (get_pf() ? FL_PF : 0) | (get_zf() ? FL_ZF : 0) |
           (get_sf() ? FL_SF : 0) | (get_of() ? FL_OF : 0) | (DF ? 0x400u : 0) | (IF ? 0x200u : 0);
}
static inline void set_eflags(uint32_t f) { fl_op = F_EXPL; fl_expl = f; DF = (f >> 10) & 1; }
static inline void set_cf(uint32_t v) { uint32_t f = get_eflags(); set_eflags(v ? (f | FL_CF) : (f & ~FL_CF)); }
static inline void set_zf(uint32_t v) { uint32_t f = get_eflags(); set_eflags(v ? (f | FL_ZF) : (f & ~FL_ZF)); }

// ---------------------------------------------------------------- ALU
static inline uint32_t alu_add(uint32_t a, uint32_t b, int sz) { uint32_t r = (a + b) & MASKSZ(sz); SETFL(F_ADD, sz, a, b, r); return r; }
static inline uint32_t alu_adc(uint32_t a, uint32_t b, int sz) { uint32_t c = get_cf(); uint32_t r = (a + b + c) & MASKSZ(sz); SETFL(F_ADC, sz, a, b, r); fl_c = c; return r; }
static inline uint32_t alu_sub(uint32_t a, uint32_t b, int sz) { uint32_t r = (a - b) & MASKSZ(sz); SETFL(F_SUB, sz, a, b, r); return r; }
static inline uint32_t alu_sbb(uint32_t a, uint32_t b, int sz) { uint32_t c = get_cf(); uint32_t r = (a - b - c) & MASKSZ(sz); SETFL(F_SBB, sz, a, b, r); fl_c = c; return r; }
static inline uint32_t alu_and(uint32_t a, uint32_t b, int sz) { uint32_t r = a & b & MASKSZ(sz); SETFL(F_LOGIC, sz, a, b, r); return r; }
static inline uint32_t alu_or(uint32_t a, uint32_t b, int sz) { uint32_t r = (a | b) & MASKSZ(sz); SETFL(F_LOGIC, sz, a, b, r); return r; }
static inline uint32_t alu_xor(uint32_t a, uint32_t b, int sz) { uint32_t r = (a ^ b) & MASKSZ(sz); SETFL(F_LOGIC, sz, a, b, r); return r; }
static inline uint32_t alu_inc(uint32_t a, int sz) { uint32_t c = get_cf(); uint32_t r = (a + 1) & MASKSZ(sz); SETFL(F_INC, sz, a, 1, r); fl_c = c; return r; }
static inline uint32_t alu_dec(uint32_t a, int sz) { uint32_t c = get_cf(); uint32_t r = (a - 1) & MASKSZ(sz); SETFL(F_DEC, sz, a, 1, r); fl_c = c; return r; }
static inline uint32_t alu_neg(uint32_t a, int sz) { uint32_t r = (0 - a) & MASKSZ(sz); SETFL(F_NEG, sz, a, 0, r); return r; }
static inline uint32_t alu_not(uint32_t a, int sz) { return ~a & MASKSZ(sz); }

static inline uint32_t alu_shl(uint32_t a, uint32_t n, int sz) {
    if (!n) return a & MASKSZ(sz);
    uint32_t r = n >= 32 ? 0 : (a << n) & MASKSZ(sz);
    uint32_t c = n > (uint32_t)sz ? 0 : (uint32_t)(((uint64_t)(a & MASKSZ(sz)) << n) >> sz) & 1;
    SETFL(F_SHL, sz, a, n, r); fl_c = c; return r;
}
static inline uint32_t alu_shr(uint32_t a, uint32_t n, int sz) {
    if (!n) return a & MASKSZ(sz);
    a &= MASKSZ(sz);
    uint32_t r = n >= 32 ? 0 : a >> n;
    uint32_t c = n > 32 ? 0 : (uint32_t)(((uint64_t)a >> (n - 1)) & 1);
    SETFL(F_SHR, sz, a, n, r); fl_c = c; return r;
}
static inline uint32_t alu_sar(uint32_t a, uint32_t n, int sz) {
    if (!n) return a & MASKSZ(sz);
    int32_t s = SXT(a, sz);
    uint32_t r = (uint32_t)(s >> (n > 31 ? 31 : n)) & MASKSZ(sz);
    uint32_t c = (uint32_t)(s >> (n - 1 > 31 ? 31 : n - 1)) & 1;
    SETFL(F_SAR, sz, a, n, r); fl_c = c; return r;
}
// rotates only touch CF/OF: materialize flags, then patch them
static inline void rot_flags(uint32_t cf, uint32_t of) {
    uint32_t f = get_eflags() & ~(FL_CF | FL_OF);
    set_eflags(f | (cf ? FL_CF : 0) | (of ? FL_OF : 0));
}
static inline uint32_t alu_rol(uint32_t a, uint32_t n, int sz) {
    a &= MASKSZ(sz);
    if (!n) return a;
    uint32_t k = n % sz; uint32_t r = k ? ((a << k) | (a >> (sz - k))) & MASKSZ(sz) : a;
    uint32_t cf = r & 1; rot_flags(cf, ((r & SIGNSZ(sz)) ? 1 : 0) ^ cf); return r;
}
static inline uint32_t alu_ror(uint32_t a, uint32_t n, int sz) {
    a &= MASKSZ(sz);
    if (!n) return a;
    uint32_t k = n % sz; uint32_t r = k ? ((a >> k) | (a << (sz - k))) & MASKSZ(sz) : a;
    uint32_t cf = (r & SIGNSZ(sz)) ? 1 : 0; rot_flags(cf, cf ^ ((r & (SIGNSZ(sz) >> 1)) ? 1 : 0)); return r;
}
static inline uint32_t alu_rcl(uint32_t a, uint32_t n, int sz) {
    a &= MASKSZ(sz);
    n %= (sz + 1);
    if (!n) return a;
    uint32_t cf = get_cf();
    for (uint32_t k = 0; k < n; k++) { uint32_t out = (a & SIGNSZ(sz)) ? 1 : 0; a = ((a << 1) | cf) & MASKSZ(sz); cf = out; }
    rot_flags(cf, ((a & SIGNSZ(sz)) ? 1 : 0) ^ cf); return a;
}
static inline uint32_t alu_rcr(uint32_t a, uint32_t n, int sz) {
    a &= MASKSZ(sz);
    n %= (sz + 1);
    if (!n) return a;
    uint32_t cf = get_cf();
    uint32_t of = ((a & SIGNSZ(sz)) ? 1 : 0) ^ cf;
    for (uint32_t k = 0; k < n; k++) { uint32_t out = a & 1; a = (a >> 1) | (cf ? SIGNSZ(sz) : 0); cf = out; }
    rot_flags(cf, of); return a;
}
static inline uint32_t alu_shld(uint32_t d, uint32_t s, uint32_t n, int sz) {
    if (!n) return d & MASKSZ(sz);
    uint64_t v = ((uint64_t)(d & MASKSZ(sz)) << sz) | (s & MASKSZ(sz));
    uint32_t r = (uint32_t)((v << n) >> sz) & MASKSZ(sz);
    uint32_t c = (uint32_t)((v << (n - 1)) >> (2 * sz - 1)) & 1;
    SETFL(F_RESCF, sz, d, ((r ^ d) & SIGNSZ(sz)) ? 1 : 0, r); fl_c = c; return r;
}
static inline uint32_t alu_shrd(uint32_t d, uint32_t s, uint32_t n, int sz) {
    if (!n) return d & MASKSZ(sz);
    uint64_t v = ((uint64_t)(s & MASKSZ(sz)) << sz) | (d & MASKSZ(sz));
    uint32_t r = (uint32_t)(v >> n) & MASKSZ(sz);
    uint32_t c = (uint32_t)(v >> (n - 1)) & 1;
    SETFL(F_RESCF, sz, d, ((r ^ d) & SIGNSZ(sz)) ? 1 : 0, r); fl_c = c; return r;
}
static inline uint32_t alu_imul(uint32_t a, uint32_t b, int sz) {
    int64_t p = (int64_t)SXT(a, sz) * (int64_t)SXT(b, sz);
    uint32_t r = (uint32_t)p & MASKSZ(sz);
    SETFL(F_MUL, sz, a, b, r); fl_c = p != (int64_t)SXT(r, sz); return r;
}
void rt_imul1(uint32_t v, int sz);
void rt_mul1(uint32_t v, int sz);
void rt_div1(uint32_t v, int sz);
void rt_idiv1(uint32_t v, int sz);

// ---------------------------------------------------------------- control transfer
typedef void (*gfn)(void);
struct fentry { uint32_t addr; gfn fn; };
struct rentry { uint32_t lo, hi; void (*fn)(uint32_t); };
extern const struct fentry FUNCS[];
extern const int NFUNCS;
extern const struct rentry REGIONS[];
extern const int NREGIONS;
extern const uint32_t GUEST_ENTRY;
void rt_dispatch(uint32_t addr);        // return address already pushed
// shadow return stack: a `ret` whose popped address differs from what the matching call pushed
// is really a jump (push addr; ret) and is routed through the dispatcher.
extern uint32_t SH[]; extern int SHP;
void rt_ret_jump(uint32_t ra);
#define CALLF(fn, nxt) do { PUSH32(nxt); SH[SHP++] = (nxt); fn(); SHP--; } while (0)
#define CALLD(t, nxt) do { uint32_t t_ = (t); PUSH32(nxt); SH[SHP++] = (nxt); rt_dispatch(t_); SHP--; } while (0)
#define RET(n) do { uint32_t ra_ = RD32(SB[2] + ESP); ESP += 4 + (n); \
                    if (__builtin_expect(ra_ != SH[SHP - 1], 0)) rt_ret_jump(ra_); } while (0)
void rt_dispatch_tail(uint32_t addr);   // jump
void rt_retf(int n);
void rt_iret(void);
void rt_farjmp(uint32_t off, uint16_t sel);
void rt_farcall(uint32_t off, uint16_t sel, uint32_t ret);

#ifdef RT_DEBUG
void rt_fn_enter(uint32_t a);
#define FN_ENTER(a) rt_fn_enter(a)
#else
#define FN_ENTER(a) ((void)0)
#endif
int snd_hle(uint32_t fn);
void rt_idle(void);
void pad_game_frame(void);
void snd_service(void);
extern int rt_skip_copyprot;   // skip the manual-lookup screen (default on; QUAR_COPYPROT=1 keeps it)
extern volatile int irq_pending;
void rt_irq(void);
#define IRQCHK() do { if (__builtin_expect(irq_pending, 0)) rt_irq(); } while (0)

// ---------------------------------------------------------------- system
void rt_int(int n);
uint32_t rt_in8(uint32_t port);
uint32_t rt_in16(uint32_t port);
uint32_t rt_in32(uint32_t port);
void rt_out8(uint32_t port, uint32_t v);
void rt_out16(uint32_t port, uint32_t v);
void rt_out32(uint32_t port, uint32_t v);
void rt_str_movs(int sz, int seg, char rep);
void rt_str_stos(int sz, int seg, char rep);
void rt_str_lods(int sz, int seg, char rep);
void rt_str_scas(int sz, int seg, char rep);
void rt_str_cmps(int sz, int seg, char rep);
void rt_str_ins(int sz, int seg, char rep);
void rt_str_outs(int sz, int seg, char rep);
int rt_lsl(uint32_t sel, uint32_t *out);
int rt_lar(uint32_t sel, uint32_t *out);

// ---------------------------------------------------------------- x87
extern double FPR[8];
extern uint32_t FPU_TOP, FPU_CW, FPU_CC;
#define ST(i) FPR[(FPU_TOP + (i)) & 7]
static inline void fpu_push(double v) { FPU_TOP = (FPU_TOP - 1) & 7; FPR[FPU_TOP] = v; }
static inline void fpu_pop(void) { FPU_TOP = (FPU_TOP + 1) & 7; }
static inline double fpu_ld_f32(uint32_t a) { uint32_t v = RD32(a); float f; memcpy(&f, &v, 4); return f; }
static inline double fpu_ld_f64(uint32_t a) { uint64_t v = RD32(a) | ((uint64_t)RD32(a + 4) << 32); double d; memcpy(&d, &v, 8); return d; }
double fpu_ld_f80(uint32_t a);
static inline double fpu_ld_i16(uint32_t a) { return (int16_t)RD16(a); }
static inline double fpu_ld_i32(uint32_t a) { return (int32_t)RD32(a); }
static inline double fpu_ld_i64(uint32_t a) { return (double)(int64_t)(RD32(a) | ((uint64_t)RD32(a + 4) << 32)); }
double fpu_round(double v);
static inline void fpu_st_f32(uint32_t a, double v) { float f = (float)v; uint32_t u; memcpy(&u, &f, 4); WR32(a, u); }
static inline void fpu_st_f64(uint32_t a, double v) { uint64_t u; memcpy(&u, &v, 8); WR32(a, (uint32_t)u); WR32(a + 4, (uint32_t)(u >> 32)); }
void fpu_st_f80(uint32_t a, double v);
static inline void fpu_st_i16(uint32_t a, double v) { double r = fpu_round(v); WR16(a, (r >= 32768.0 || r < -32768.0) ? 0x8000 : (uint16_t)(int16_t)r); }
static inline void fpu_st_i32(uint32_t a, double v) { double r = fpu_round(v); WR32(a, (r >= 2147483648.0 || r < -2147483648.0) ? 0x80000000u : (uint32_t)(int32_t)r); }
static inline void fpu_st_i64(uint32_t a, double v) { int64_t r = (int64_t)fpu_round(v); WR32(a, (uint32_t)r); WR32(a + 4, (uint32_t)((uint64_t)r >> 32)); }
static inline void fpu_compare(double a, double b) { FPU_CC = (a > b) ? 0 : (a < b) ? 0x0100 : (a == b) ? 0x4000 : 0x4500; }
static inline uint32_t fpu_sw(void) { return FPU_CC | ((FPU_TOP & 7) << 11); }
void fpu_init(void);
void fpu_save(uint32_t a);
void fpu_restore(uint32_t a);
