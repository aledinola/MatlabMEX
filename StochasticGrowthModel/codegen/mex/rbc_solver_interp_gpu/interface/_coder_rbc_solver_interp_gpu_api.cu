//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_rbc_solver_interp_gpu_api.cu
//
// Code generation for function '_coder_rbc_solver_interp_gpu_api'
//

// Include files
#include "_coder_rbc_solver_interp_gpu_api.h"
#include "rbc_solver_interp_gpu.h"
#include "rbc_solver_interp_gpu_data.h"
#include "rbc_solver_interp_gpu_types.h"
#include "rt_nonfinite.h"

// Function Declarations
static real_T *b_emlrt_marshallIn(const mxArray *b_nullptr,
                                  const char_T *identifier, int32_T y_size[1]);

static real_T *b_emlrt_marshallIn(const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T y_size[1]);

static real_T b_emlrt_marshallIn(const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T *c_emlrt_marshallIn(const mxArray *b_nullptr,
                                  const char_T *identifier, int32_T y_size[2]);

static real_T *c_emlrt_marshallIn(const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T y_size[2]);

static real_T *d_emlrt_marshallIn(const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[2]);

static real_T *e_emlrt_marshallIn(const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[1]);

static real_T *emlrt_marshallIn(const mxArray *b_nullptr,
                                const char_T *identifier, int32_T y_size[2]);

static real_T *emlrt_marshallIn(const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                int32_T y_size[2]);

static real_T emlrt_marshallIn(const mxArray *b_nullptr,
                               const char_T *identifier);

static real_T emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(real_T u_data[],
                                        const int32_T u_size[2]);

static real_T *f_emlrt_marshallIn(const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[2]);

// Function Definitions
static real_T *b_emlrt_marshallIn(const mxArray *b_nullptr,
                                  const char_T *identifier, int32_T y_size[1])
{
  emlrtMsgIdentifier thisId;
  real_T *y_data;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y_data = b_emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId, y_size);
  emlrtDestroyArray(&b_nullptr);
  return y_data;
}

static real_T *b_emlrt_marshallIn(const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T y_size[1])
{
  real_T *y_data;
  y_data = e_emlrt_marshallIn(emlrtAlias(u), parentId, y_size);
  emlrtDestroyArray(&u);
  return y_data;
}

static real_T b_emlrt_marshallIn(const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static real_T *c_emlrt_marshallIn(const mxArray *b_nullptr,
                                  const char_T *identifier, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  real_T *y_data;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y_data = c_emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId, y_size);
  emlrtDestroyArray(&b_nullptr);
  return y_data;
}

static real_T *c_emlrt_marshallIn(const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  int32_T y_size[2])
{
  real_T *y_data;
  y_data = f_emlrt_marshallIn(emlrtAlias(u), parentId, y_size);
  emlrtDestroyArray(&u);
  return y_data;
}

static real_T *d_emlrt_marshallIn(const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[2])
{
  static const int32_T dims[2]{2000, 100};
  real_T *ret_data;
  int32_T iv[2];
  boolean_T bv[2]{true, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  ret_data = static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret_data;
}

static real_T *e_emlrt_marshallIn(const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[1])
{
  static const int32_T dims[1]{2000};
  real_T *ret_data;
  int32_T iv[1];
  boolean_T bv[1]{true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_data = static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret_data;
}

static real_T *emlrt_marshallIn(const mxArray *b_nullptr,
                                const char_T *identifier, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  real_T *y_data;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y_data = emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId, y_size);
  emlrtDestroyArray(&b_nullptr);
  return y_data;
}

static real_T *emlrt_marshallIn(const mxArray *u,
                                const emlrtMsgIdentifier *parentId,
                                int32_T y_size[2])
{
  real_T *y_data;
  y_data = d_emlrt_marshallIn(emlrtAlias(u), parentId, y_size);
  emlrtDestroyArray(&u);
  return y_data;
}

static real_T emlrt_marshallIn(const mxArray *b_nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = b_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(real_T u_data[],
                                        const int32_T u_size[2])
{
  static const int32_T iv[2]{0, 0};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u_size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T *f_emlrt_marshallIn(const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  int32_T ret_size[2])
{
  static const int32_T dims[2]{100, 100};
  real_T *ret_data;
  int32_T iv[2];
  boolean_T bv[2]{true, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  ret_data = static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret_data;
}

void rbc_solver_interp_gpu_api(rbc_solver_interp_gpuStackData *SD,
                               const mxArray *const prhs[4], int32_T nlhs,
                               const mxArray *plhs[2])
{
  real_T(*c0_data)[200000];
  real_T(*pol_kp_data)[200000];
  real_T(*v_data)[200000];
  real_T(*pdfz_data)[10000];
  real_T(*k_data)[2000];
  real_T tol;
  int32_T c0_size[2];
  int32_T pdfz_size[2];
  int32_T pol_kp_size[2];
  int32_T v_size[2];
  int32_T k_size[1];
  v_data = (real_T(*)[200000])mxMalloc(sizeof(real_T[200000]));
  pol_kp_data = (real_T(*)[200000])mxMalloc(sizeof(real_T[200000]));
  // Marshall function inputs
  *(real_T **)&c0_data = emlrt_marshallIn(emlrtAlias(prhs[0]), "c0", c0_size);
  *(real_T **)&k_data = b_emlrt_marshallIn(emlrtAlias(prhs[1]), "k", k_size);
  *(real_T **)&pdfz_data =
      c_emlrt_marshallIn(emlrtAlias(prhs[2]), "pdfz", pdfz_size);
  tol = emlrt_marshallIn(emlrtAliasP(prhs[3]), "tol");
  // Invoke the target function
  rbc_solver_interp_gpu(SD, *c0_data, c0_size, *k_data, k_size, *pdfz_data,
                        pdfz_size, tol, *v_data, v_size, *pol_kp_data,
                        pol_kp_size);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*v_data, v_size);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(*pol_kp_data, pol_kp_size);
  }
}

// End of code generation (_coder_rbc_solver_interp_gpu_api.cu)
