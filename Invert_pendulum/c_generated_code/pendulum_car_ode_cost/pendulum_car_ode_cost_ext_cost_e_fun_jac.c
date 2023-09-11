/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) pendulum_car_ode_cost_ext_cost_e_fun_jac_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[16] = {1., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

/* pendulum_car_ode_cost_ext_cost_e_fun_jac:(i0[4],i1[],i2[],i3[])->(o0,o1[4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, *w1=w+2, w2, w3, w4, w5, *w6=w+10, *w7=w+14, *w8=w+18, *w9=w+34;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = zeros(1x4) */
  casadi_clear(w1, 4);
  /* #2: @2 = input[0][0] */
  w2 = arg[0] ? arg[0][0] : 0;
  /* #3: @3 = input[0][1] */
  w3 = arg[0] ? arg[0][1] : 0;
  /* #4: @4 = input[0][2] */
  w4 = arg[0] ? arg[0][2] : 0;
  /* #5: @5 = input[0][3] */
  w5 = arg[0] ? arg[0][3] : 0;
  /* #6: @6 = vertcat(@2, @3, @4, @5) */
  rr=w6;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  /* #7: @7 = @6' */
  casadi_copy(w6, 4, w7);
  /* #8: @8 = 
  [[1, 0, 0, 0], 
   [0, 2, 0, 0], 
   [0, 0, 0, 0], 
   [0, 0, 0, 0]] */
  casadi_copy(casadi_c0, 16, w8);
  /* #9: @1 = mac(@7,@8,@1) */
  for (i=0, rr=w1; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w8+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #10: @0 = mac(@1,@6,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w6+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #11: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #12: @1 = @1' */
  /* #13: @7 = zeros(1x4) */
  casadi_clear(w7, 4);
  /* #14: @6 = @6' */
  /* #15: @9 = @8' */
  for (i=0, rr=w9, cs=w8; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #16: @7 = mac(@6,@9,@7) */
  for (i=0, rr=w7; i<4; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w6+j, tt=w9+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #17: @7 = @7' */
  /* #18: @1 = (@1+@7) */
  for (i=0, rr=w1, cs=w7; i<4; ++i) (*rr++) += (*cs++);
  /* #19: output[1][0] = @1 */
  casadi_copy(w1, 4, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int pendulum_car_ode_cost_ext_cost_e_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int pendulum_car_ode_cost_ext_cost_e_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int pendulum_car_ode_cost_ext_cost_e_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void pendulum_car_ode_cost_ext_cost_e_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int pendulum_car_ode_cost_ext_cost_e_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void pendulum_car_ode_cost_ext_cost_e_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void pendulum_car_ode_cost_ext_cost_e_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void pendulum_car_ode_cost_ext_cost_e_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int pendulum_car_ode_cost_ext_cost_e_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int pendulum_car_ode_cost_ext_cost_e_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real pendulum_car_ode_cost_ext_cost_e_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pendulum_car_ode_cost_ext_cost_e_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pendulum_car_ode_cost_ext_cost_e_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pendulum_car_ode_cost_ext_cost_e_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pendulum_car_ode_cost_ext_cost_e_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int pendulum_car_ode_cost_ext_cost_e_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 8;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 50;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
