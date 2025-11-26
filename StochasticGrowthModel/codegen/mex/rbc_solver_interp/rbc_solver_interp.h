/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rbc_solver_interp.h
 *
 * Code generation for function 'rbc_solver_interp'
 *
 */

#pragma once

/* Include files */
#include "rbc_solver_interp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

void rbc_solver_interp(const emlrtStack *sp, const emxArray_real_T *c0,
                       const emxArray_real_T *k, const emxArray_real_T *pdfz,
                       real_T tol, emxArray_real_T *v, emxArray_real_T *pol_kp);

real_T rbc_solver_interp_anonFcn1(const emlrtStack *sp, real_T wealth,
                                  const emxArray_real_T *k,
                                  const emxArray_real_T *EV_z, real_T x);

/* End of code generation (rbc_solver_interp.h) */
