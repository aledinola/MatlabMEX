/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_rbc_solver_interp_api.c
 *
 * Code generation for function '_coder_rbc_solver_interp_api'
 *
 */

/* Include files */
#include "_coder_rbc_solver_interp_api.h"
#include "rbc_solver_interp.h"
#include "rbc_solver_interp_data.h"
#include "rbc_solver_interp_emxutil.h"
#include "rbc_solver_interp_mexutil.h"
#include "rbc_solver_interp_types.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRTEInfo v_emlrtRTEI = {
    1,                              /* lineNo */
    1,                              /* colNo */
    "_coder_rbc_solver_interp_api", /* fName */
    ""                              /* pName */
};

/* Function Declarations */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, emxArray_real_T *y);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, emxArray_real_T *y);

static const mxArray *emlrt_marshallOut(emxArray_real_T *u);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret);

/* Function Definitions */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
  h_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static const mxArray *emlrt_marshallOut(emxArray_real_T *u)
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  real_T *u_data;
  u_data = u->data;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 2);
  u->canFreeData = false;
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               emxArray_real_T *y)
{
  i_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
  static const int32_T dims[2] = {-1, -1};
  int32_T iv[2];
  int32_T i;
  boolean_T bv[2] = {true, true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(sp, ret, i, (emlrtRTEInfo *)NULL);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               emxArray_real_T *ret)
{
  static const int32_T dims = -1;
  int32_T i;
  int32_T i1;
  boolean_T b = true;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret->allocatedSize = i;
  i1 = ret->size[0];
  ret->size[0] = i;
  emxEnsureCapacity_real_T(sp, ret, i1, (emlrtRTEInfo *)NULL);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

void rbc_solver_interp_api(const mxArray *const prhs[4], int32_T nlhs,
                           const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  emxArray_real_T *c0;
  emxArray_real_T *k;
  emxArray_real_T *pdfz;
  emxArray_real_T *pol_kp;
  emxArray_real_T *v;
  real_T tol;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  /* Marshall function inputs */
  emxInit_real_T(&st, &c0, 2, &v_emlrtRTEI);
  c0->canFreeData = false;
  c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "c0", c0);
  emxInit_real_T(&st, &k, 1, &v_emlrtRTEI);
  k->canFreeData = false;
  e_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "k", k);
  emxInit_real_T(&st, &pdfz, 2, &v_emlrtRTEI);
  pdfz->canFreeData = false;
  c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "pdfz", pdfz);
  tol = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "tol");
  /* Invoke the target function */
  emxInit_real_T(&st, &v, 2, &v_emlrtRTEI);
  emxInit_real_T(&st, &pol_kp, 2, &v_emlrtRTEI);
  rbc_solver_interp(&st, c0, k, pdfz, tol, v, pol_kp);
  emxFree_real_T(&st, &pdfz);
  emxFree_real_T(&st, &k);
  emxFree_real_T(&st, &c0);
  /* Marshall function outputs */
  v->canFreeData = false;
  plhs[0] = emlrt_marshallOut(v);
  emxFree_real_T(&st, &v);
  if (nlhs > 1) {
    pol_kp->canFreeData = false;
    plhs[1] = emlrt_marshallOut(pol_kp);
  }
  emxFree_real_T(&st, &pol_kp);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_rbc_solver_interp_api.c) */
