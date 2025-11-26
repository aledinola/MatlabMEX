/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rbc_solver_interp_types.h
 *
 * Code generation for function 'rbc_solver_interp'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  emxArray_real_T *k;
  emxArray_real_T *EV_z;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_anonymous_function
#define typedef_anonymous_function
typedef struct {
  struct_T workspace;
} anonymous_function;
#endif /* typedef_anonymous_function */

/* End of code generation (rbc_solver_interp_types.h) */
