/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rbc_solver_interp.c
 *
 * Code generation for function 'rbc_solver_interp'
 *
 */

/* Include files */
#include "rbc_solver_interp.h"
#include "brent_min.h"
#include "eml_int_forloop_overflow_check.h"
#include "mtimes.h"
#include "rbc_solver_interp_data.h"
#include "rbc_solver_interp_emxutil.h"
#include "rbc_solver_interp_mexutil.h"
#include "rbc_solver_interp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = {
    52,                  /* lineNo */
    9,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRSInfo emlrtRSI = {
    22,                  /* lineNo */
    "rbc_solver_interp", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    39,                  /* lineNo */
    "rbc_solver_interp", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    46,                  /* lineNo */
    "rbc_solver_interp", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    52,                  /* lineNo */
    "rbc_solver_interp", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI =
    {
        94,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI =
    {
        69,                  /* lineNo */
        "eml_mtimes_helper", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    38,                                     /* lineNo */
    "@(x)rhs_bellman(x,wealth,k,EV_z,eta)", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    68,            /* lineNo */
    "rhs_bellman", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    69,            /* lineNo */
    "rhs_bellman", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    18,             /* lineNo */
    "interp1_scal", /* fcnName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI =
    {
        19,    /* lineNo */
        "abs", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elfun\\abs.m" /* pathName
                                                                          */
};

static emlrtRSInfo v_emlrtRSI = {
    79,                    /* lineNo */
    "applyScalarFunction", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\applyScalarFunction.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    15,    /* lineNo */
    "max", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\max.m" /* pathName
                                                                        */
};

static emlrtRSInfo x_emlrtRSI =
    {
        64,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo y_emlrtRSI =
    {
        99,        /* lineNo */
        "maximum", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo ab_emlrtRSI = {
    290,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    383,                     /* lineNo */
    "unaryMinOrMaxDispatch", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    451,          /* lineNo */
    "minOrMax2D", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    533,                         /* lineNo */
    "minOrMax2DColumnMajorDim1", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    531,                         /* lineNo */
    "minOrMax2DColumnMajorDim1", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    255,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    966,                    /* lineNo */
    "maxRealVectorOmitNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI = {
    73,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    65,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo jb_emlrtRSI = {
    114,         /* lineNo */
    "findFirst", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo kb_emlrtRSI = {
    131,                        /* lineNo */
    "minOrMaxRealVectorKernel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    38,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

static emlrtMCInfo emlrtMCI = {
    66,        /* lineNo */
    18,        /* colNo */
    "fprintf", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pName
                                                                          */
};

static emlrtRTEInfo b_emlrtRTEI =
    {
        133,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI =
    {
        138,                   /* lineNo */
        23,                    /* colNo */
        "dynamic_size_checks", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    198,             /* lineNo */
    27,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    90,              /* lineNo */
    27,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    2,                   /* nDims */
    46,                  /* lineNo */
    24,                  /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    1,                   /* nDims */
    46,                  /* lineNo */
    24,                  /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    25,                  /* lineNo */
    21,                  /* colNo */
    "ev",                /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    11,                  /* lineNo */
    11,                  /* colNo */
    "k",                 /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    12,                  /* lineNo */
    11,                  /* colNo */
    "k",                 /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    28,                  /* lineNo */
    25,                  /* colNo */
    "c0",                /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    28,                  /* lineNo */
    28,                  /* colNo */
    "c0",                /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    40,                  /* lineNo */
    16,                  /* colNo */
    "v0",                /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    40,                  /* lineNo */
    19,                  /* colNo */
    "v0",                /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    41,                  /* lineNo */
    20,                  /* colNo */
    "pol_kp",            /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    41,                  /* lineNo */
    23,                  /* colNo */
    "pol_kp",            /* aName */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    21,             /* lineNo */
    12,             /* colNo */
    "y",            /* aName */
    "interp1_scal", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    21,             /* lineNo */
    19,             /* colNo */
    "y",            /* aName */
    "interp1_scal", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    21,             /* lineNo */
    26,             /* colNo */
    "x",            /* aName */
    "interp1_scal", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    21,             /* lineNo */
    33,             /* colNo */
    "x",            /* aName */
    "interp1_scal", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    22,             /* lineNo */
    8,              /* colNo */
    "y",            /* aName */
    "interp1_scal", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    22,             /* lineNo */
    17,             /* colNo */
    "x",            /* aName */
    "interp1_scal", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\inter"
    "p1_scal.m", /* pName */
    0            /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = {
    -1,       /* iFirst */
    -1,       /* iLast */
    12,       /* lineNo */
    9,        /* colNo */
    "xx",     /* aName */
    "locate", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\locat"
    "e.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    -1,       /* iFirst */
    -1,       /* iLast */
    14,       /* lineNo */
    13,       /* colNo */
    "xx",     /* aName */
    "locate", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\locat"
    "e.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,       /* iFirst */
    -1,       /* iLast */
    21,       /* lineNo */
    18,       /* colNo */
    "xx",     /* aName */
    "locate", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\locat"
    "e.m", /* pName */
    0      /* checkKind */
};

static emlrtRTEInfo g_emlrtRTEI = {
    14,                  /* lineNo */
    1,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    16,                  /* lineNo */
    1,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo i_emlrtRTEI =
    {
        76,                  /* lineNo */
        9,                   /* colNo */
        "eml_mtimes_helper", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_"
        "helper.m" /* pName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    15,                  /* lineNo */
    6,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    30,                    /* lineNo */
    21,                    /* colNo */
    "applyScalarFunction", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\applyScalarFunction.m" /* pName */
};

static emlrtRTEInfo l_emlrtRTEI = {
    25,                  /* lineNo */
    9,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo m_emlrtRTEI = {
    38,                  /* lineNo */
    13,                  /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo n_emlrtRTEI = {
    523,             /* lineNo */
    21,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtRTEInfo o_emlrtRTEI = {
    56,                  /* lineNo */
    5,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo p_emlrtRTEI = {
    15,                  /* lineNo */
    1,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI = {
    22,                  /* lineNo */
    5,                   /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo r_emlrtRTEI = {
    22,                  /* lineNo */
    10,                  /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    46,                  /* lineNo */
    16,                  /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    46,                  /* lineNo */
    24,                  /* colNo */
    "rbc_solver_interp", /* fName */
    "C:"
    "\\Users\\aledi\\Documents\\GitHub\\MatlabMEX\\StochasticGrowthModel\\rbc_"
    "solver_interp.m" /* pName */
};

static emlrtRSInfo mb_emlrtRSI = {
    66,        /* lineNo */
    "fprintf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\iofun\\fprintf.m" /* pathName
                                                                          */
};

/* Function Declarations */
static const mxArray *feval(const emlrtStack *sp, const mxArray *m1,
                            const mxArray *m2, const mxArray *m3,
                            const mxArray *m4, const mxArray *m5,
                            emlrtMCInfo *location);

static void minus(const emlrtStack *sp, emxArray_real_T *in1,
                  const emxArray_real_T *in2);

/* Function Definitions */
static const mxArray *feval(const emlrtStack *sp, const mxArray *m1,
                            const mxArray *m2, const mxArray *m3,
                            const mxArray *m4, const mxArray *m5,
                            emlrtMCInfo *location)
{
  const mxArray *pArrays[5];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  pArrays[2] = m3;
  pArrays[3] = m4;
  pArrays[4] = m5;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 5, &pArrays[0],
                               "feval", true, location);
}

static void minus(const emlrtStack *sp, emxArray_real_T *in1,
                  const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &w_emlrtRTEI);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = loop_ub;
  if (in2->size[1] == 1) {
    b_loop_ub = in1->size[1];
  } else {
    b_loop_ub = in2->size[1];
  }
  b_in1->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &w_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_0_1 = (in1->size[1] != 1);
  stride_1_0 = (in2->size[0] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[0] * i] =
          in1_data[i1 * stride_0_0 + in1->size[0] * aux_0_1] -
          in2_data[i1 * stride_1_0 + in2->size[0] * aux_1_1];
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  in1->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(sp, in1, i, &w_emlrtRTEI);
  in1_data = in1->data;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
    }
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

void rbc_solver_interp(const emlrtStack *sp, const emxArray_real_T *c0,
                       const emxArray_real_T *k, const emxArray_real_T *pdfz,
                       real_T tol, emxArray_real_T *v, emxArray_real_T *pol_kp)
{
  static const int32_T iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 22};
  static const char_T b_u[22] = {'i', 't', 's', ' ', '=',  ' ', '%', 'd',
                                 ',', ' ', 'd', 'i', 'f',  'f', ' ', '=',
                                 ' ', '%', 'f', ' ', '\\', 'n'};
  static const char_T u[7] = {'f', 'p', 'r', 'i', 'n', 't', 'f'};
  __m128d r;
  __m128d r1;
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  anonymous_function myfun;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack st;
  emxArray_real_T *ev;
  emxArray_real_T *maxval;
  emxArray_real_T *v0;
  emxArray_real_T *y;
  const mxArray *b_m;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const real_T *c0_data;
  const real_T *k_data;
  real_T a;
  real_T diff;
  real_T fval;
  real_T k_max;
  real_T k_min;
  real_T xf;
  real_T *ev_data;
  real_T *pol_kp_data;
  real_T *v0_data;
  real_T *v_data;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T ik;
  int32_T its;
  int32_T iz;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  int32_T m;
  int32_T nk;
  int32_T nz;
  int32_T rbc_solver_interp_numThreads;
  int32_T scalarLB_tmp;
  int32_T unnamed_idx_0;
  int32_T unnamed_idx_1;
  int32_T vectorUB_tmp;
  boolean_T emlrtHadParallelError = false;
  boolean_T exitg1;
  boolean_T overflow;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  d_st.prev = &b_st;
  d_st.tls = b_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_data = k->data;
  c0_data = c0->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  diff = 1.0;
  its = 1;
  /*  Discount rate */
  nk = c0->size[0];
  nz = c0->size[1] - 1;
  if (k->size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, k->size[0], &b_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  k_min = k_data[0];
  if ((c0->size[0] < 1) || (c0->size[0] > k->size[0])) {
    emlrtDynamicBoundsCheckR2012b(c0->size[0], 1, k->size[0], &c_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  k_max = k_data[c0->size[0] - 1];
  i = v->size[0] * v->size[1];
  v->size[0] = c0->size[0];
  v->size[1] = c0->size[1];
  emxEnsureCapacity_real_T(sp, v, i, &g_emlrtRTEI);
  v_data = v->data;
  loop_ub_tmp = c0->size[0] * c0->size[1];
  for (i = 0; i < loop_ub_tmp; i++) {
    v_data[i] = 0.0;
  }
  /*  Value Function */
  unnamed_idx_0 = c0->size[0];
  unnamed_idx_1 = c0->size[1];
  /*  v at the previous iteration */
  i = pol_kp->size[0] * pol_kp->size[1];
  pol_kp->size[0] = c0->size[0];
  pol_kp->size[1] = c0->size[1];
  emxEnsureCapacity_real_T(sp, pol_kp, i, &h_emlrtRTEI);
  pol_kp_data = pol_kp->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    pol_kp_data[i] = 0.0;
  }
  emxInit_real_T(sp, &v0, 2, &p_emlrtRTEI);
  emxInit_real_T(sp, &ev, 2, &q_emlrtRTEI);
  emxInit_real_T(sp, &y, 2, &r_emlrtRTEI);
  emxInit_real_T(sp, &maxval, 2, &s_emlrtRTEI);
  while ((diff > tol) && (its < 2000)) {
    /*  In Fortran it would be: */
    /*  EV(k',z) = matmul(v(k',z'),transpose(pdf(z,z'))) */
    i = v->size[0];
    i1 = y->size[0] * y->size[1];
    y->size[0] = v->size[0];
    m = v->size[1];
    y->size[1] = v->size[1];
    emxEnsureCapacity_real_T(sp, y, i1, &i_emlrtRTEI);
    ev_data = y->data;
    loop_ub_tmp = v->size[0] * v->size[1];
    scalarLB_tmp = (loop_ub_tmp / 2) << 1;
    vectorUB_tmp = scalarLB_tmp - 2;
    for (i1 = 0; i1 <= vectorUB_tmp; i1 += 2) {
      r = _mm_loadu_pd(&v_data[i1]);
      _mm_storeu_pd(&ev_data[i1], _mm_mul_pd(_mm_set1_pd(0.95), r));
    }
    for (i1 = scalarLB_tmp; i1 < loop_ub_tmp; i1++) {
      ev_data[i1] = 0.95 * v_data[i1];
    }
    st.site = &emlrtRSI;
    b_st.site = &f_emlrtRSI;
    if (y->size[1] != pdfz->size[1]) {
      if (((y->size[0] == 1) && (y->size[1] == 1)) ||
          ((pdfz->size[0] == 1) && (pdfz->size[1] == 1))) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &b_emlrtRTEI,
            "Coder:toolbox:mtimes_noDynamicScalarExpansion",
            "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&b_st, &c_emlrtRTEI, "MATLAB:innerdim",
                                      "MATLAB:innerdim", 0);
      }
    }
    b_st.site = &e_emlrtRSI;
    mtimes(&b_st, y, pdfz, ev);
    ev_data = ev->data;
    i1 = v0->size[0] * v0->size[1];
    v0->size[0] = unnamed_idx_0;
    v0->size[1] = unnamed_idx_1;
    emxEnsureCapacity_real_T(sp, v0, i1, &j_emlrtRTEI);
    v0_data = v0->data;
    i1 = pol_kp->size[0] * pol_kp->size[1];
    pol_kp->size[0] = unnamed_idx_0;
    pol_kp->size[1] = unnamed_idx_1;
    emxEnsureCapacity_real_T(sp, pol_kp, i1, &j_emlrtRTEI);
    pol_kp_data = pol_kp->data;
    emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
    emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    rbc_solver_interp_numThreads = emlrtAllocRegionTLSs(
        sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(rbc_solver_interp_numThreads) private(        \
        myfun, fval, xf, emlrtJBEnviron, k_st, loop_ub, i2, ik)                \
    firstprivate(c_st, emlrtHadParallelError)
    {
      if (setjmp(emlrtJBEnviron) == 0) {
        c_st.prev = sp;
        c_st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
        c_st.site = NULL;
        emlrtSetJmpBuf(&c_st, &emlrtJBEnviron);
        k_st.prev = &c_st;
        k_st.tls = c_st.tls;
        c_emxInitStruct_anonymous_funct(&c_st, &myfun, &m_emlrtRTEI);
      } else {
        emlrtHadParallelError = true;
      }
#pragma omp for nowait
      for (iz = 0; iz <= nz; iz++) {
        if (emlrtHadParallelError) {
          continue;
        }
        if (setjmp(emlrtJBEnviron) == 0) {
          /*  z today (exogenous state) */
          if (iz + 1 > ev->size[1]) {
            emlrtDynamicBoundsCheckR2012b(iz + 1, 1, ev->size[1], &emlrtBCI,
                                          &c_st);
          }
          loop_ub = ev->size[0];
          i2 = myfun.workspace.EV_z->size[0];
          myfun.workspace.EV_z->size[0] = ev->size[0];
          emxEnsureCapacity_real_T(&c_st, myfun.workspace.EV_z, i2,
                                   &l_emlrtRTEI);
          for (i2 = 0; i2 < loop_ub; i2++) {
            myfun.workspace.EV_z->data[i2] = ev_data[i2 + ev->size[0] * iz];
          }
          loop_ub = k->size[0];
          for (ik = 0; ik < nk; ik++) {
            /*  k today (endo state) */
            if (ik + 1 > c0->size[0]) {
              emlrtDynamicBoundsCheckR2012b(ik + 1, 1, c0->size[0], &d_emlrtBCI,
                                            &c_st);
            }
            if (iz + 1 > c0->size[1]) {
              emlrtDynamicBoundsCheckR2012b(iz + 1, 1, c0->size[1], &e_emlrtBCI,
                                            &c_st);
            }
            xf = c0_data[ik + c0->size[0] * iz];
            /*  Impose k' in [k_min, min(k_max, wealth)] */
            fval = muDoubleScalarMin(k_max, xf) - 1.0E-8;
            /*  If wealth is extremely small, enforce a strictly positive
             * interval */
            if (fval <= k_min) {
              fval = k_min + 1.0E-8;
            }
            i2 = myfun.workspace.k->size[0];
            myfun.workspace.k->size[0] = k->size[0];
            emxEnsureCapacity_real_T(&c_st, myfun.workspace.k, i2,
                                     &m_emlrtRTEI);
            for (i2 = 0; i2 < loop_ub; i2++) {
              myfun.workspace.k->data[i2] = k_data[i2];
            }
            k_st.site = &b_emlrtRSI;
            xf = brent_min(&k_st, xf, myfun.workspace.k, myfun.workspace.EV_z,
                           k_min, fval, &fval);
            if (ik + 1 > v0->size[0]) {
              emlrtDynamicBoundsCheckR2012b(ik + 1, 1, v0->size[0], &f_emlrtBCI,
                                            &c_st);
            }
            if (iz + 1 > v0->size[1]) {
              emlrtDynamicBoundsCheckR2012b(iz + 1, 1, v0->size[1], &g_emlrtBCI,
                                            &c_st);
            }
            v0_data[ik + v0->size[0] * iz] = -fval;
            if (ik + 1 > pol_kp->size[0]) {
              emlrtDynamicBoundsCheckR2012b(ik + 1, 1, pol_kp->size[0],
                                            &h_emlrtBCI, &c_st);
            }
            if (iz + 1 > pol_kp->size[1]) {
              emlrtDynamicBoundsCheckR2012b(iz + 1, 1, pol_kp->size[1],
                                            &i_emlrtBCI, &c_st);
            }
            pol_kp_data[ik + pol_kp->size[0] * iz] = xf;
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(&c_st);
            }
          }
          /* end ik */
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(&c_st);
          }
        } else {
          emlrtHadParallelError = true;
        }
      }
      if (!emlrtHadParallelError) {
        emlrtHeapReferenceStackLeaveScope(&c_st, 1);
        c_emxFreeStruct_anonymous_funct(&c_st, &myfun);
      }
    }
    emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
    emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
    /* end iz */
    if ((i != v0->size[0]) && ((i != 1) && (v0->size[0] != 1))) {
      emlrtDimSizeImpxCheckR2021b(i, v0->size[0], &b_emlrtECI,
                                  (emlrtConstCTX)sp);
    }
    if ((m != v0->size[1]) && ((m != 1) && (v0->size[1] != 1))) {
      emlrtDimSizeImpxCheckR2021b(m, v0->size[1], &emlrtECI, (emlrtConstCTX)sp);
    }
    st.site = &c_emlrtRSI;
    b_st.site = &c_emlrtRSI;
    if ((v->size[0] == v0->size[0]) && (v->size[1] == v0->size[1])) {
      for (i = 0; i <= vectorUB_tmp; i += 2) {
        r = _mm_loadu_pd(&v_data[i]);
        r1 = _mm_loadu_pd(&v0_data[i]);
        _mm_storeu_pd(&v_data[i], _mm_sub_pd(r, r1));
      }
      for (i = scalarLB_tmp; i < loop_ub_tmp; i++) {
        v_data[i] -= v0_data[i];
      }
    } else {
      d_st.site = &c_emlrtRSI;
      minus(&d_st, v, v0);
      v_data = v->data;
    }
    d_st.site = &u_emlrtRSI;
    m = v->size[0] * v->size[1];
    i = y->size[0] * y->size[1];
    y->size[0] = v->size[0];
    y->size[1] = v->size[1];
    emxEnsureCapacity_real_T(&d_st, y, i, &k_emlrtRTEI);
    ev_data = y->data;
    e_st.site = &v_emlrtRSI;
    if (m > 2147483646) {
      f_st.site = &h_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }
    for (vectorUB_tmp = 0; vectorUB_tmp < m; vectorUB_tmp++) {
      ev_data[vectorUB_tmp] = muDoubleScalarAbs(v_data[vectorUB_tmp]);
    }
    b_st.site = &w_emlrtRSI;
    d_st.site = &x_emlrtRSI;
    e_st.site = &y_emlrtRSI;
    if (((y->size[0] != 1) || (y->size[1] != 1)) && (y->size[0] == 1)) {
      emlrtErrorWithMessageIdR2018a(&e_st, &e_emlrtRTEI,
                                    "Coder:toolbox:autoDimIncompatibility",
                                    "Coder:toolbox:autoDimIncompatibility", 0);
    }
    if (y->size[0] < 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &d_emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }
    f_st.site = &ab_emlrtRSI;
    g_st.site = &bb_emlrtRSI;
    h_st.site = &cb_emlrtRSI;
    m = y->size[0];
    scalarLB_tmp = y->size[1];
    i = maxval->size[0] * maxval->size[1];
    maxval->size[0] = 1;
    maxval->size[1] = y->size[1];
    emxEnsureCapacity_real_T(&h_st, maxval, i, &n_emlrtRTEI);
    pol_kp_data = maxval->data;
    if (y->size[1] >= 1) {
      i_st.site = &eb_emlrtRSI;
      if (y->size[1] > 2147483646) {
        j_st.site = &h_emlrtRSI;
        check_forloop_overflow_error(&j_st);
      }
      overflow = (y->size[0] > 2147483646);
      for (loop_ub_tmp = 0; loop_ub_tmp < scalarLB_tmp; loop_ub_tmp++) {
        pol_kp_data[loop_ub_tmp] = ev_data[y->size[0] * loop_ub_tmp];
        i_st.site = &db_emlrtRSI;
        if (overflow) {
          j_st.site = &h_emlrtRSI;
          check_forloop_overflow_error(&j_st);
        }
        for (vectorUB_tmp = 2; vectorUB_tmp <= m; vectorUB_tmp++) {
          a = pol_kp_data[loop_ub_tmp];
          diff = ev_data[(vectorUB_tmp + y->size[0] * loop_ub_tmp) - 1];
          if (muDoubleScalarIsNaN(diff)) {
            p = false;
          } else if (muDoubleScalarIsNaN(a)) {
            p = true;
          } else {
            p = (a < diff);
          }
          if (p) {
            pol_kp_data[loop_ub_tmp] = diff;
          }
        }
      }
    }
    st.site = &c_emlrtRSI;
    b_st.site = &w_emlrtRSI;
    d_st.site = &x_emlrtRSI;
    e_st.site = &y_emlrtRSI;
    if (maxval->size[1] < 1) {
      emlrtErrorWithMessageIdR2018a(
          &e_st, &d_emlrtRTEI, "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }
    f_st.site = &fb_emlrtRSI;
    g_st.site = &gb_emlrtRSI;
    if (maxval->size[1] <= 2) {
      if (maxval->size[1] == 1) {
        diff = pol_kp_data[0];
      } else if ((pol_kp_data[0] < pol_kp_data[1]) ||
                 (muDoubleScalarIsNaN(pol_kp_data[0]) &&
                  (!muDoubleScalarIsNaN(pol_kp_data[1])))) {
        diff = pol_kp_data[1];
      } else {
        diff = pol_kp_data[0];
      }
    } else {
      h_st.site = &ib_emlrtRSI;
      if (!muDoubleScalarIsNaN(pol_kp_data[0])) {
        m = 1;
      } else {
        m = 0;
        i_st.site = &jb_emlrtRSI;
        if (maxval->size[1] > 2147483646) {
          j_st.site = &h_emlrtRSI;
          check_forloop_overflow_error(&j_st);
        }
        vectorUB_tmp = 2;
        exitg1 = false;
        while ((!exitg1) && (vectorUB_tmp <= scalarLB_tmp)) {
          if (!muDoubleScalarIsNaN(pol_kp_data[vectorUB_tmp - 1])) {
            m = vectorUB_tmp;
            exitg1 = true;
          } else {
            vectorUB_tmp++;
          }
        }
      }
      if (m == 0) {
        diff = pol_kp_data[0];
      } else {
        h_st.site = &hb_emlrtRSI;
        diff = pol_kp_data[m - 1];
        loop_ub_tmp = m + 1;
        i_st.site = &kb_emlrtRSI;
        if ((m + 1 <= maxval->size[1]) && (maxval->size[1] > 2147483646)) {
          j_st.site = &h_emlrtRSI;
          check_forloop_overflow_error(&j_st);
        }
        for (vectorUB_tmp = loop_ub_tmp; vectorUB_tmp <= scalarLB_tmp;
             vectorUB_tmp++) {
          a = pol_kp_data[vectorUB_tmp - 1];
          if (diff < a) {
            diff = a;
          }
        }
      }
    }
    /*  Check convergence: */
    /* if mod(its, 60) == 0 */
    /*     fprintf('%5.0f ~ %8.10f \n', its, diff); */
    /* end */
    if (its - its / 60 * 60 == 0) {
      emlrtAssertMATLABThread((emlrtCTX)sp, &emlrtRTEI);
      emlrtAssertMATLABThread((emlrtCTX)sp, &emlrtRTEI);
      st.site = &d_emlrtRSI;
      b_st.site = &lb_emlrtRSI;
      b_y = NULL;
      b_m = emlrtCreateCharArray(2, &iv[0]);
      emlrtInitCharArrayR2013a(&b_st, 7, b_m, &u[0]);
      emlrtAssign(&b_y, b_m);
      c_y = NULL;
      b_m = emlrtCreateDoubleScalar(1.0);
      emlrtAssign(&c_y, b_m);
      d_y = NULL;
      b_m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(&b_st, 22, b_m, &b_u[0]);
      emlrtAssign(&d_y, b_m);
      e_y = NULL;
      b_m = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
      *(int32_T *)emlrtMxGetData(b_m) = its;
      emlrtAssign(&e_y, b_m);
      f_y = NULL;
      b_m = emlrtCreateDoubleScalar(diff);
      emlrtAssign(&f_y, b_m);
      d_st.site = &mb_emlrtRSI;
      a = emlrt_marshallIn(&d_st,
                           feval(&d_st, b_y, c_y, d_y, e_y, f_y, &emlrtMCI),
                           "<output of feval>");
      g_y = NULL;
      b_m = emlrtCreateDoubleScalar(a);
      emlrtAssign(&g_y, b_m);
      emlrtDisplayR2012b(g_y, "ans", &emlrtRTEI, (emlrtCTX)sp);
    }
    /*  Update */
    i = v->size[0] * v->size[1];
    v->size[0] = v0->size[0];
    v->size[1] = v0->size[1];
    emxEnsureCapacity_real_T(sp, v, i, &o_emlrtRTEI);
    v_data = v->data;
    loop_ub_tmp = v0->size[0] * v0->size[1];
    for (i = 0; i < loop_ub_tmp; i++) {
      v_data[i] = v0_data[i];
    }
    its++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  emxFree_real_T(sp, &maxval);
  emxFree_real_T(sp, &y);
  emxFree_real_T(sp, &ev);
  emxFree_real_T(sp, &v0);
  /* end while */
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

real_T rbc_solver_interp_anonFcn1(const emlrtStack *sp, real_T wealth,
                                  const emxArray_real_T *k,
                                  const emxArray_real_T *EV_z, real_T x)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  const real_T *EV_z_data;
  const real_T *k_data;
  real_T varargout_1;
  real_T varargout_1_tmp;
  int32_T i;
  int32_T j;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  EV_z_data = EV_z->data;
  k_data = k->data;
  st.site = &o_emlrtRSI;
  /* end function */
  /*  %--------------------------- SUBFUNCTIONS
   * --------------------------------% */
  /*  This calls user-written function interp1_scal */
  b_st.site = &p_emlrtRSI;
  /*  Fast linear interpolation routine */
  /*  Usage: */
  /*  yi = interp1_scal(x,y,xi) */
  /*  where x and y are column vectors with n elements, xi is a scalar and yi */
  /*  is a scalar */
  /*  Input Arguments */
  /*  x - Sample points */
  /*    column vector */
  /*  Y - Sample data */
  /*    column vector */
  /*  xi - Query point */
  /*    scalar */
  c_st.site = &r_emlrtRSI;
  /* function jl = locate(xx,x) */
  /*  */
  /*  x is between xx(jl) and xx(jl+1) */
  /*  */
  /*  jl = 0 and jl = n means x is out of range */
  /*  */
  /*  xx is assumed to be monotone increasing */
  i = k->size[0];
  if (k->size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, k->size[0], &p_emlrtBCI, &c_st);
  }
  if (x < k_data[0]) {
    j = 0;
  } else {
    if (k->size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(k->size[0], 1, k->size[0], &q_emlrtBCI,
                                    &c_st);
    }
    if (x > k_data[k->size[0] - 1]) {
      j = k->size[0];
    } else {
      int32_T ju;
      j = 1;
      ju = k->size[0];
      while (ju - j > 1) {
        int32_T jm;
        jm = (int32_T)muDoubleScalarFloor((real_T)((uint32_T)ju + (uint32_T)j) /
                                          2.0);
        if ((jm < 1) || (jm > i)) {
          emlrtDynamicBoundsCheckR2012b(jm, 1, i, &r_emlrtBCI, &c_st);
        }
        if (x >= k_data[jm - 1]) {
          j = jm;
        } else {
          ju = jm;
        }
      }
    }
  }
  j = (int32_T)muDoubleScalarMax(muDoubleScalarMin(j, (real_T)k->size[0] - 1.0),
                                 1.0);
  if (j + 1 > EV_z->size[0]) {
    emlrtDynamicBoundsCheckR2012b(j + 1, 1, EV_z->size[0], &j_emlrtBCI, &b_st);
  }
  if (j > EV_z->size[0]) {
    emlrtDynamicBoundsCheckR2012b(j, 1, EV_z->size[0], &k_emlrtBCI, &b_st);
  }
  if (j + 1 > k->size[0]) {
    emlrtDynamicBoundsCheckR2012b(j + 1, 1, k->size[0], &l_emlrtBCI, &b_st);
  }
  if (j > k->size[0]) {
    emlrtDynamicBoundsCheckR2012b(j, 1, k->size[0], &m_emlrtBCI, &b_st);
  }
  if (j > EV_z->size[0]) {
    emlrtDynamicBoundsCheckR2012b(j, 1, EV_z->size[0], &n_emlrtBCI, &b_st);
  }
  if (j > k->size[0]) {
    emlrtDynamicBoundsCheckR2012b(j, 1, k->size[0], &o_emlrtBCI, &b_st);
  }
  b_st.site = &q_emlrtRSI;
  varargout_1 = EV_z_data[j - 1];
  varargout_1_tmp = k_data[j - 1];
  return -(
      -muDoubleScalarPower(wealth - x, -1.0) +
      (varargout_1 + (x - varargout_1_tmp) * ((EV_z_data[j] - varargout_1) /
                                              (k_data[j] - varargout_1_tmp))));
}

/* End of code generation (rbc_solver_interp.c) */
