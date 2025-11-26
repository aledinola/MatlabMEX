//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rbc_solver_interp_gpu_types.h
//
// Code generation for function 'rbc_solver_interp_gpu'
//

#pragma once

// Include files
#include "rtwtypes.h"
#include "emlrt.h"

// Custom Header Code

#ifdef __CUDA_ARCH__
#undef printf
#endif

// Type Definitions
struct b_rbc_solver_interp_gpu {
  real_T cpu_v0_data[200000];
  real_T cpu_y_data[200000];
  real_T cpu_x_data[200000];
};

struct rbc_solver_interp_gpuStackData {
  b_rbc_solver_interp_gpu f0;
};

// End of code generation (rbc_solver_interp_gpu_types.h)
