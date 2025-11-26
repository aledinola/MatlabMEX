/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rbc_solver_interp_terminate.c
 *
 * Code generation for function 'rbc_solver_interp_terminate'
 *
 */

/* Include files */
#include "rbc_solver_interp_terminate.h"
#include "_coder_rbc_solver_interp_mex.h"
#include "rbc_solver_interp_data.h"
#include "rt_nonfinite.h"
#include "omp.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void rbc_solver_interp_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&rbc_solver_interp_nestLockGlobal);
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&rbc_solver_interp_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&rbc_solver_interp_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

void rbc_solver_interp_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (rbc_solver_interp_terminate.c) */
