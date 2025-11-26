/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * brent_min.c
 *
 * Code generation for function 'brent_min'
 *
 */

/* Include files */
#include "brent_min.h"
#include "rbc_solver_interp.h"
#include "rbc_solver_interp_data.h"
#include "rbc_solver_interp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo j_emlrtRSI = {
    28,          /* lineNo */
    "brent_min", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\brent"
    "_min.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    29,          /* lineNo */
    "brent_min", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\brent"
    "_min.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    43,          /* lineNo */
    "brent_min", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\brent"
    "_min.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    112,         /* lineNo */
    "brent_min", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\brent"
    "_min.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    63,                               /* lineNo */
    "function_handle/parenReference", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\function_"
    "handle.m" /* pathName */
};

/* Function Definitions */
real_T brent_min(const emlrtStack *sp, real_T funfcn_workspace_wealth,
                 const emxArray_real_T *funfcn_workspace_k,
                 const emxArray_real_T *funfcn_workspace_EV_z, real_T ax,
                 real_T bx, real_T *fval)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T a;
  real_T b;
  real_T d;
  real_T e;
  real_T fv;
  real_T fw;
  real_T tol1;
  real_T tol2;
  real_T v;
  real_T w;
  real_T xf;
  real_T xm;
  int32_T funccount;
  int32_T iter;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  /*  brent_min: Single-variable bounded function minimization (Brent's method)
   */
  /*  without derivatives. */
  /*  */
  /*  INPUTS */
  /*    funfcn  : function handle to the scalar function to be minimized */
  /*    ax      : lower bound of the interval */
  /*    bx      : upper bound of the interval */
  /*    tol     : tolerance criterion for X */
  /*    maxfun  : maximum number of function evaluations */
  /*    maxiter : maximum number of iterations */
  /*  */
  /*  OUTPUTS */
  /*    xf       : minimizing x */
  /*    fval     : function value at xf */
  /*    exitflag : 1 if converged, 0 if maxfun or maxiter reached */
  iter = 0;
  /*  Assume convergence unless we hit limits */
  /*  Constants */
  st.site = &j_emlrtRSI;
  st.site = &k_emlrtRSI;
  /*  0.381966011250105... */
  /*  Initial bracketing */
  a = ax;
  b = bx;
  v = ax + 0.3819660112501051 * (bx - ax);
  w = v;
  xf = v;
  d = 0.0;
  e = 0.0;
  st.site = &l_emlrtRSI;
  b_st.site = &n_emlrtRSI;
  *fval =
      rbc_solver_interp_anonFcn1(&b_st, funfcn_workspace_wealth,
                                 funfcn_workspace_k, funfcn_workspace_EV_z, v);
  funccount = 1;
  fv = *fval;
  fw = *fval;
  xm = 0.5 * (ax + bx);
  tol1 = 1.4901161193847656E-8 * muDoubleScalarAbs(v) + 3.3333333333333334E-9;
  tol2 = 2.0 * tol1;
  /*  Main loop */
  exitg1 = false;
  while ((!exitg1) && (muDoubleScalarAbs(xf - xm) > tol2 - 0.5 * (b - a))) {
    real_T p;
    real_T x;
    boolean_T guard1;
    /*  flag for golden-section step */
    /*  Attempt parabolic step if possible */
    guard1 = false;
    if (muDoubleScalarAbs(e) > tol1) {
      real_T q;
      real_T r;
      x = xf - w;
      r = x * (*fval - fv);
      p = xf - v;
      q = p * (*fval - fw);
      p = p * q - x * r;
      q = 2.0 * (q - r);
      if (q > 0.0) {
        p = -p;
      }
      q = muDoubleScalarAbs(q);
      r = e;
      e = d;
      /*  Check whether the parabolic step is acceptable */
      if ((muDoubleScalarAbs(p) < muDoubleScalarAbs(0.5 * q * r)) &&
          (p > q * (a - xf)) && (p < q * (b - xf))) {
        /*  Parabolic interpolation step */
        d = p / q;
        x = xf + d;
        /*  f must not be evaluated too close to the interval endpoints */
        if ((x - a < tol2) || (b - x < tol2)) {
          if (xm >= xf) {
            d = tol1;
          } else {
            d = -tol1;
          }
        }
      } else {
        /*  Reject parabola, do golden-section step */
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      /*  Golden-section step */
      if (xf >= xm) {
        e = a - xf;
      } else {
        e = b - xf;
      }
      d = 0.3819660112501051 * e;
    }
    /*  The function must not be evaluated too close to xf */
    if (d >= 0.0) {
      x = xf + muDoubleScalarMax(d, tol1);
    } else {
      x = xf + muDoubleScalarMin(d, -tol1);
    }
    st.site = &m_emlrtRSI;
    b_st.site = &n_emlrtRSI;
    p = rbc_solver_interp_anonFcn1(&b_st, funfcn_workspace_wealth,
                                   funfcn_workspace_k, funfcn_workspace_EV_z,
                                   x);
    funccount++;
    iter++;
    /*  Update a, b, v, w, x, xm, tol1, tol2 */
    if (p <= *fval) {
      if (x >= xf) {
        a = xf;
      } else {
        b = xf;
      }
      v = w;
      fv = fw;
      w = xf;
      fw = *fval;
      xf = x;
      *fval = p;
    } else {
      if (x < xf) {
        a = x;
      } else {
        b = x;
      }
      if ((p <= fw) || (w == xf)) {
        v = w;
        fv = fw;
        w = x;
        fw = p;
      } else if ((p <= fv) || (v == xf) || (v == w)) {
        v = x;
        fv = p;
      }
    }
    xm = 0.5 * (a + b);
    tol1 =
        1.4901161193847656E-8 * muDoubleScalarAbs(xf) + 3.3333333333333334E-9;
    tol2 = 2.0 * tol1;
    /*  Stopping due to limits */
    if ((funccount >= 500) || (iter >= 500)) {
      exitg1 = true;
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  return xf;
}

/* End of code generation (brent_min.c) */
