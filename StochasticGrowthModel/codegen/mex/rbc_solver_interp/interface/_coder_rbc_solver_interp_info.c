/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_rbc_solver_interp_info.c
 *
 * Code generation for function 'rbc_solver_interp'
 *
 */

/* Include files */
#include "_coder_rbc_solver_interp_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[6] = {
      "789cdd56cb6ed3401475a0a022f1081bf62cd944142912b0a3496a68e588d44943a9c11d"
      "db53d9e978c68f7148fa05ec58c137b0ec923d1bb66cd8c007b0e403"
      "40e0f891d896468e64e34abe9b9bab939973eebdc9b1b9c633a1c171dc4d2e8cf3db61be"
      "11d5cd285fe2d291c51b51be9ca9e3b8c26da4cec5f8bb28ab045338",
      "a361818109972735621a18603a9c5b9073a04bd0146a0172622038344c28268bfea23277"
      "12d0b258408bcf1d1daaa7a267728eeeae14a264b19cc72746bf1b6b"
      "cee380318f66063feabdea3c96462e745c0920a8195297a89e09317525dea04f3d451200"
      "4540117a2f24911255072e3554de216fa82e100d22c9515439988e23",
      "1bfe301dab65aefab00af6712da78f18571c5fb2ec6f2caa63feb705f9efe6f0c7b8ea8f"
      "c26905fd63805a00133c3789e7ca271e56a941b2bace0beaeae7e88a"
      "f112f6bb1c6d72afc70c7db7d6d49fcdabef6f06f9efeb1f015415df57ebdbb84abe382e"
      "8a6fc6b86fdddfdf1d065f3383cfc6e31eb5b7ee0fda2fed8793e174",
      "0e77dd07bd958ee7393c793a38465dd5fd5f18e7d79da3c2b8bf99c1ff8f4f37fc2791ac"
      "40844c80d37d15f5edeb397dc578a8644b76558092fcc705f9af32f9"
      "4344239e826079cfdb01932f8d97b0c7e4c816961c44553ef2ab625fbef7f3e87d957c71"
      "d4dd97476ebb6fefb7796bdb120622ec9ed9fb22cfd7c797ebfe7f2e",
      "cb071151018517e7bb45df83f7987c69bc843d85a38a3654953f7cb0bf57eab77f3e7ffc"
      "5d255f1c75f7db8976d6998c0e0ef7b671f791fe64ea1ddade4e0dfc"
      "f61f9e5848be",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 4552U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 4);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("rbc_solver_interp"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("C:"
                          "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\Stocha"
                          "sticGrowthModel\\rbc_solver_interp.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739947.74969907408));
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

/* End of code generation (_coder_rbc_solver_interp_info.c) */
