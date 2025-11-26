/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rbc_solver_interp_initialize.c
 *
 * Code generation for function 'rbc_solver_interp_initialize'
 *
 */

/* Include files */
#include "rbc_solver_interp_initialize.h"
#include "_coder_rbc_solver_interp_mex.h"
#include "rbc_solver_interp_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void rbc_solver_interp_once(void);

/* Function Definitions */
static void rbc_solver_interp_once(void)
{
  mex_InitInfAndNan();
}

void rbc_solver_interp_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    rbc_solver_interp_once();
  }
}

/* End of code generation (rbc_solver_interp_initialize.c) */
