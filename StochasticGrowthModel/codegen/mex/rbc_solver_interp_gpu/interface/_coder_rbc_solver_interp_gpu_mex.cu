//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_rbc_solver_interp_gpu_mex.cu
//
// Code generation for function '_coder_rbc_solver_interp_gpu_mex'
//

// Include files
#include "_coder_rbc_solver_interp_gpu_mex.h"
#include "_coder_rbc_solver_interp_gpu_api.h"
#include "rbc_solver_interp_gpu_data.h"
#include "rbc_solver_interp_gpu_initialize.h"
#include "rbc_solver_interp_gpu_terminate.h"
#include "rbc_solver_interp_gpu_types.h"
#include "rt_nonfinite.h"
#include <stdexcept>

void emlrtExceptionBridge();
void emlrtExceptionBridge()
{
  throw std::runtime_error("");
}
// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  rbc_solver_interp_gpuStackData *c_rbc_solver_interp_gpuStackDat{nullptr};
  c_rbc_solver_interp_gpuStackDat =
      static_cast<rbc_solver_interp_gpuStackData *>(
          new rbc_solver_interp_gpuStackData);
  mexAtExit(&rbc_solver_interp_gpu_atexit);
  rbc_solver_interp_gpu_initialize();
  try {
    unsafe_rbc_solver_interp_gpu_mexFunction(c_rbc_solver_interp_gpuStackDat,
                                             nlhs, plhs, nrhs, prhs);
    rbc_solver_interp_gpu_terminate();
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
  delete c_rbc_solver_interp_gpuStackDat;
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           (void *)&emlrtExceptionBridge, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_rbc_solver_interp_gpu_mexFunction(
    rbc_solver_interp_gpuStackData *SD, int32_T nlhs, mxArray *plhs[2],
    int32_T nrhs, const mxArray *prhs[4])
{
  const mxArray *outputs[2];
  int32_T b;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 4, 4, 21, "rbc_solver_interp_gpu");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 21,
                        "rbc_solver_interp_gpu");
  }
  // Call the function.
  rbc_solver_interp_gpu_api(SD, prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    b = 1;
  } else {
    b = nlhs;
  }
  emlrtReturnArrays(b, &plhs[0], &outputs[0]);
}

// End of code generation (_coder_rbc_solver_interp_gpu_mex.cu)
