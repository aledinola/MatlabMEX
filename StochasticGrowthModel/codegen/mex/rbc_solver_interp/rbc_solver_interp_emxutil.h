/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rbc_solver_interp_emxutil.h
 *
 * Code generation for function 'rbc_solver_interp_emxutil'
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
void c_emxFreeStruct_anonymous_funct(const emlrtStack *sp,
                                     anonymous_function *pStruct);

void c_emxInitStruct_anonymous_funct(const emlrtStack *sp,
                                     anonymous_function *pStruct,
                                     const emlrtRTEInfo *srcLocation);

void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxFreeStruct_struct_T(const emlrtStack *sp, struct_T *pStruct);

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

void emxInitStruct_struct_T(const emlrtStack *sp, struct_T *pStruct,
                            const emlrtRTEInfo *srcLocation);

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    int32_T numDimensions, const emlrtRTEInfo *srcLocation);

/* End of code generation (rbc_solver_interp_emxutil.h) */
