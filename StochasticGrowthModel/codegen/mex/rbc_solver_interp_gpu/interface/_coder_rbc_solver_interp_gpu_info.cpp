//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_rbc_solver_interp_gpu_info.cpp
//
// Code generation for function 'rbc_solver_interp_gpu'
//

// Include files
#include "_coder_rbc_solver_interp_gpu_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *c_emlrtMexFcnResolvedFunctionsI();

// Function Definitions
static const mxArray *c_emlrtMexFcnResolvedFunctionsI()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[6]{
      "789cdd56bb8ed340149dc08240e2111a7a4a9a88458a04746c9235b07244d60961770d66"
      "6c4fb0b3e3193fc621d92fa0a3826fa0a4a444a2a1a5a1810fa0e403"
      "40e0d899c4b63472a404af94db5c5f1dcf9c33f7dac7069587720500700524f1f15a922f"
      "cfeaea2c9f01d9c8e395593e9bab799c035b99751c7f33cb06250c8d",
      "595210e8a0f94a933a36818475272e023e0a281e2133460636465ddb414aba684f2b6737"
      "05cd8b2934bd6e58c838564207f856b05088d3c5bc1f9f04e7dd5ab2"
      "1f87827e5473f851eb59e39eda0b901fa81023d3569bd4081d4458a04a367b10eaaa0c19"
      "86badc7aaa2a8c1a160c986d483e7dc52c999a08abbe6e6871777ccd",
      "8e9ae9bbda4b37ac39d9f3b82b9ee762c17938aefb91742d9a1cc8f2bf5e91ff46013fc7"
      "8da8257e2dee0381b806092513878681360889c16c9ad7f561455ded"
      "025d1c5fc39ce7ad9dce96eb7f21d0777549fdf9bcb8ff429cff3eff114365f17d75bff5"
      "cbe4e3715a7c63c17ecb3e7fd7057cd51c3eeef75bccdbbed5a91f7a",
      "7786ddd1043d0a6eb7163a1e17f014e90082baacfdbf08d62fdbc78160ff6a0eff7f7e5d"
      "89be4a9a8e307620599f6f5f2a3817c71325db5a60403cad8bde8765"
      "f9cf0bf913c4a4a18e51ea3f6445be8e902f8baf618ee996f1cf6d693ef2ab645fbef9f3"
      "e86d997c3c36dd977b41bdededd72577c7953b0a6a9e78fb8a246d8e",
      "2f6ffafbbc2e1fc4d4800c9d9eefaefa1fbc27e4cbe26b9853d2aad984caf28777def752"
      "fdf6cfe7f7bfcbe4e3b1e97e3b344f1ac3de9383bd1dd2bc6bdd1f85"
      "075eb8bb017efb0f74144c34",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 4560U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties()
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9]{"Version",
                                 "ResolvedFunctions",
                                 "Checksum",
                                 "EntryPoints",
                                 "CoverageInfo",
                                 "IsPolymorphic",
                                 "PropertyList",
                                 "UUID",
                                 "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8]{
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 4);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("rbc_solver_interp_gpu"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("C:"
                          "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\Stocha"
                          "sticGrowthModel\\rbc_solver_interp_gpu.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739947.74986111117));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2923080 (R2024b) Update 6"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("RPkXMoF7UwXVsAsGEGMbEB"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

// End of code generation (_coder_rbc_solver_interp_gpu_info.cpp)
