/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_rbc_solver_interp_mex.c
 *
 * Code generation for function '_coder_rbc_solver_interp_mex'
 *
 */

/* Include files */
#include "_coder_rbc_solver_interp_mex.h"
#include "_coder_rbc_solver_interp_api.h"
#include "rbc_solver_interp.h"
#include "rbc_solver_interp_data.h"
#include "rbc_solver_interp_initialize.h"
#include "rbc_solver_interp_terminate.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&rbc_solver_interp_atexit);
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&rbc_solver_interp_nestLockGlobal);
  rbc_solver_interp_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    rbc_solver_interp_mexFunction(nlhs, plhs, nrhs, prhs);
    rbc_solver_interp_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&rbc_solver_interp_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&rbc_solver_interp_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void rbc_solver_interp_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                                   const mxArray *prhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        17, "rbc_solver_interp");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 17,
                        "rbc_solver_interp");
  }
  /* Call the function. */
  rbc_solver_interp_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_rbc_solver_interp_mex.c) */
