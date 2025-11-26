//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rbc_solver_interp_gpu.h
//
// Code generation for function 'rbc_solver_interp_gpu'
//

#pragma once

// Include files
#include "rbc_solver_interp_gpu_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Custom Header Code

#ifdef __CUDA_ARCH__
#undef printf
#endif

// Function Declarations
void rbc_solver_interp_gpu(rbc_solver_interp_gpuStackData *SD,
                           const real_T cpu_c0_data[], const int32_T c0_size[2],
                           const real_T cpu_k_data[], const int32_T k_size[1],
                           const real_T cpu_pdfz_data[],
                           const int32_T pdfz_size[2], real_T tol,
                           real_T cpu_v_data[], int32_T v_size[2],
                           real_T cpu_pol_kp_data[], int32_T pol_kp_size[2]);

// End of code generation (rbc_solver_interp_gpu.h)
