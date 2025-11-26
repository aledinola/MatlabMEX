/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * brent_min.h
 *
 * Code generation for function 'brent_min'
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
real_T brent_min(const emlrtStack *sp, real_T funfcn_workspace_wealth,
                 const emxArray_real_T *funfcn_workspace_k,
                 const emxArray_real_T *funfcn_workspace_EV_z, real_T ax,
                 real_T bx, real_T *fval);

/* End of code generation (brent_min.h) */
