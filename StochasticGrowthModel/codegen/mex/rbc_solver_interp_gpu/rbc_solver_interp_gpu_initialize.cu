//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rbc_solver_interp_gpu_initialize.cu
//
// Code generation for function 'rbc_solver_interp_gpu_initialize'
//

// Include files
#include "rbc_solver_interp_gpu_initialize.h"
#include "_coder_rbc_solver_interp_gpu_mex.h"
#include "rbc_solver_interp_gpu_data.h"
#include "rt_nonfinite.h"
#include "MWCUBLASUtils.hpp"

// Function Definitions
void rbc_solver_interp_gpu_initialize()
{
  mex_InitInfAndNan();
  emlrtInitGPU(emlrtRootTLSGlobal);
  cudaGetLastError();
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLicenseCheckR2022a(emlrtRootTLSGlobal,
                          "EMLRT:runTime:MexFunctionNeedsLicense",
                          "distrib_computing_toolbox", 2);
  cublasEnsureInitialization(CUBLAS_POINTER_MODE_HOST);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

// End of code generation (rbc_solver_interp_gpu_initialize.cu)
