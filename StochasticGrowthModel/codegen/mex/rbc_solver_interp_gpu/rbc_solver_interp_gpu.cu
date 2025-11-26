//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rbc_solver_interp_gpu.cu
//
// Code generation for function 'rbc_solver_interp_gpu'
//

// Include files
#include "rbc_solver_interp_gpu.h"
#include "rbc_solver_interp_gpu_data.h"
#include "rbc_solver_interp_gpu_types.h"
#include "rt_nonfinite.h"
#include "MWCUBLASUtils.hpp"
#include "MWCudaDimUtility.hpp"
#include "MWCudaMemoryFunctions.hpp"
#include "MWLaunchParametersUtilities.hpp"
#include <cmath>

// Function Declarations
static void binary_expand_op(real_T in1_data[], int32_T in1_size[1],
                             const real_T in2_data[], const int32_T in2_size[2],
                             const real_T in3_data[],
                             const int32_T in3_size[2]);

static void checkCudaError(cudaError_t errorCode, const char_T *file,
                           int32_T b_line);

static uint64_T computeNumIters(int32_T ub, int32_T b_ub);

static uint64_T computeNumIters(int32_T ub);

static void gpuThrowError(const char_T *file, int32_T b_line);

static void gpuThrowError(uint32_T errorCode, const char_T *errorName,
                          const char_T *errorString, const char_T *file,
                          int32_T b_line);

static __global__ void rbc_solver_interp_gpu_kernel1(const real_T k_data[],
                                                     real_T *k_min);

static __global__ void rbc_solver_interp_gpu_kernel2(
    const real_T pdfz_data[], const int32_T pdfz_size_dim0,
    const int32_T B_size_dim0, const int32_T pdfz_size,
    const int32_T b_pdfz_size, real_T B_data[10000]);

static __global__ void rbc_solver_interp_gpu_kernel3(const int32_T v_size,
                                                     real_T y_data[200000]);

static __global__ void
rbc_solver_interp_gpu_kernel4(const real_T y_data[200000], const int32_T b,
                              real_T ev_data[200000]);

static __global__ void rbc_solver_interp_gpu_kernel5(
    const int32_T pol_kp_size_dim0, const int32_T v0_size_dim0,
    const real_T ev_data[200000], const int32_T ev_size_dim0,
    const int32_T k_size_dim0, const real_T k_data[], const real_T *k_min,
    const real_T k_max, const real_T c0_data[], const int32_T c0_size_dim0,
    const int32_T nk, const int32_T b, real_T pol_kp_data[],
    real_T v0_data[200000]);

static __global__ void
rbc_solver_interp_gpu_kernel6(const real_T v0_data[200000],
                              const real_T v_data[], const int32_T b,
                              real_T x_data[200000]);

static __global__ void
rbc_solver_interp_gpu_kernel7(const real_T x_data[200000], const int32_T b,
                              real_T y_data[200000]);

static __global__ void
rbc_solver_interp_gpu_kernel8(const real_T v0_data[200000],
                              const int32_T v0_size, real_T v_data[]);

// Function Definitions
static void binary_expand_op(real_T in1_data[], int32_T in1_size[1],
                             const real_T in2_data[], const int32_T in2_size[2],
                             const real_T in3_data[], const int32_T in3_size[2])
{
  int32_T in2_idx_0;
  int32_T in3_idx_0;
  int32_T stride_0_0;
  int32_T stride_1_0;
  in2_idx_0 = in2_size[0] * in2_size[1];
  in3_idx_0 = in3_size[0] * in3_size[1];
  if (in3_idx_0 == 1) {
    in1_size[0] = in2_idx_0;
  } else {
    in1_size[0] = in3_idx_0;
  }
  stride_0_0 = (in2_idx_0 != 1);
  stride_1_0 = (in3_idx_0 != 1);
  if (in3_idx_0 == 1) {
    in3_idx_0 = in2_idx_0;
  }
  for (in2_idx_0 = 0; in2_idx_0 < in3_idx_0; in2_idx_0++) {
    in1_data[in2_idx_0] =
        in2_data[in2_idx_0 * stride_0_0] - in3_data[in2_idx_0 * stride_1_0];
  }
}

static void checkCudaError(cudaError_t errorCode, const char_T *file,
                           int32_T b_line)
{
  if (errorCode != cudaSuccess) {
    gpuThrowError(errorCode, cudaGetErrorName(errorCode),
                  cudaGetErrorString(errorCode), file, b_line);
  }
}

static uint64_T computeNumIters(int32_T ub, int32_T b_ub)
{
  uint64_T n;
  uint64_T numIters;
  boolean_T overflow;
  overflow = false;
  n = 0ULL;
  if (ub >= 0) {
    n = static_cast<uint64_T>(ub + 1);
  }
  numIters = n;
  n = 0ULL;
  if (b_ub >= 0) {
    n = static_cast<uint64_T>(b_ub + 1);
    overflow = (numIters > MAX_uint64_T / n);
  }
  numIters *= n;
  if (overflow) {
    gpuThrowError(__FILE__, __LINE__);
  }
  return numIters;
}

static uint64_T computeNumIters(int32_T ub)
{
  uint64_T numIters;
  numIters = 0ULL;
  if (ub >= 0) {
    numIters = static_cast<uint64_T>(ub + 1);
  }
  return numIters;
}

static void gpuThrowError(const char_T *file, int32_T b_line)
{
  emlrtRTEInfo rtInfo;
  rtInfo.lineNo = b_line;
  rtInfo.colNo = 0;
  rtInfo.fName = "";
  rtInfo.pName = file;
  emlrtCUDAError(
      0U, (char_T *)"_",
      (char_T
           *)"Unable to launch kernel. Loop nest contains too many iterations.",
      &rtInfo, emlrtRootTLSGlobal);
}

static void gpuThrowError(uint32_T errorCode, const char_T *errorName,
                          const char_T *errorString, const char_T *file,
                          int32_T b_line)
{
  emlrtRTEInfo rtInfo;
  rtInfo.lineNo = b_line;
  rtInfo.colNo = 0;
  rtInfo.fName = "";
  rtInfo.pName = file;
  emlrtCUDAError(errorCode, (char_T *)errorName, (char_T *)errorString, &rtInfo,
                 emlrtRootTLSGlobal);
}

static __global__ __launch_bounds__(32, 1) void rbc_solver_interp_gpu_kernel1(
    const real_T k_data[], real_T *k_min)
{
  int32_T tmpIdx;
  tmpIdx = static_cast<int32_T>(mwGetGlobalThreadIndex());
  if (tmpIdx < 1) {
    *k_min = k_data[0];
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel2(
    const real_T pdfz_data[], const int32_T pdfz_size_dim0,
    const int32_T B_size_dim0, const int32_T pdfz_size,
    const int32_T b_pdfz_size, real_T B_data[10000])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<uint64_T>(b_pdfz_size) + 1ULL) *
                (static_cast<uint64_T>(pdfz_size) + 1ULL) -
            1ULL;
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    int32_T b_idx;
    int32_T jm;
    b_idx =
        static_cast<int32_T>(idx % (static_cast<uint64_T>(pdfz_size) + 1ULL));
    jm = static_cast<int32_T>((idx - static_cast<uint64_T>(b_idx)) /
                              (static_cast<uint64_T>(pdfz_size) + 1ULL));
    B_data[b_idx + B_size_dim0 * jm] = pdfz_data[jm + pdfz_size_dim0 * b_idx];
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel3(
    const int32_T v_size, real_T y_data[200000])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<uint64_T>(v_size);
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    int32_T jm;
    jm = static_cast<int32_T>(idx);
    y_data[jm] = 0.0;
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel4(
    const real_T y_data[200000], const int32_T b, real_T ev_data[200000])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<uint64_T>(b);
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    int32_T jm;
    jm = static_cast<int32_T>(idx);
    ev_data[jm] = 0.95 * y_data[jm];
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel5(
    const int32_T pol_kp_size_dim0, const int32_T v0_size_dim0,
    const real_T ev_data[200000], const int32_T ev_size_dim0,
    const int32_T k_size_dim0, const real_T k_data[], const real_T *k_min,
    const real_T k_max, const real_T c0_data[], const int32_T c0_size_dim0,
    const int32_T nk, const int32_T b, real_T pol_kp_data[],
    real_T v0_data[200000])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<uint64_T>(b);
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    real_T a;
    real_T d;
    real_T e;
    real_T fv;
    real_T fw;
    real_T fx;
    real_T ik;
    real_T kp_ub;
    real_T tol1;
    real_T tol2;
    real_T v;
    real_T w;
    real_T wealth;
    real_T xf;
    real_T xm;
    real_T y;
    int32_T b_idx;
    int32_T funccount;
    int32_T j;
    int32_T jm;
    int32_T ju;
    boolean_T exitg1;
    b_idx = static_cast<int32_T>(idx);
    //  Recover indices for exogenous (iz) and endogenous (ik) states
    y = floor(((static_cast<real_T>(b_idx) + 1.0) - 1.0) /
              static_cast<real_T>(nk));
    //  z index
    ik = (static_cast<real_T>(b_idx) + 1.0) -
         ((y + 1.0) - 1.0) * static_cast<real_T>(nk);
    //  k index
    //  continuation value over k' for this z
    wealth = c0_data[(static_cast<int32_T>(ik) +
                      c0_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)) -
                     1];
    //  total resources today
    //  Impose k' in [k_min, min(k_max, wealth)]
    kp_ub = fmin(k_max, wealth) - 1.0E-8;
    //  If wealth is extremely small, enforce a strictly positive interval
    if (fmin(k_max, wealth) - 1.0E-8 <= *k_min) {
      kp_ub = *k_min + 1.0E-8;
    }
    //  Brent minimization over k'
    //  brent_min: Single-variable bounded function minimization (Brent's
    //  method) without derivatives.
    //
    //  INPUTS
    //    funfcn  : function handle to the scalar function to be minimized
    //    ax      : lower bound of the interval
    //    bx      : upper bound of the interval
    //    tol     : tolerance criterion for X
    //    maxfun  : maximum number of function evaluations
    //    maxiter : maximum number of iterations
    //
    //  OUTPUTS
    //    xf       : minimizing x
    //    fval     : function value at xf
    //    exitflag : 1 if converged, 0 if maxfun or maxiter reached
    b_idx = 0;
    //  Assume convergence unless we hit limits
    //  Constants
    //  0.381966011250105...
    //  Initial bracketing
    a = *k_min;
    v = *k_min + 0.3819660112501051 * (kp_ub - *k_min);
    w = v;
    xf = v;
    d = 0.0;
    e = 0.0;
    //  function rbc_solver_interp_gpu
    //  -------------------------------------------------------------------------
    //  RHS of the Bellman equation for given k' (used inside Brent).
    //  This calls user-written function interp1_scal for interpolation.
    //
    //  Returns negative of the objective, since brent_min performs
    //  minimization. Consumption implied by k' Interpolate EV_z at k' Fast
    //  linear interpolation routine Usage: yi = interp1_scal(x,y,xi) where x
    //  and y are column vectors with n elements, xi is a scalar and yi is a
    //  scalar Input Arguments x - Sample points
    //    column vector
    //  Y - Sample data
    //    column vector
    //  xi - Query point
    //    scalar
    // function jl = locate(xx,x)
    //
    //  x is between xx(jl) and xx(jl+1)
    //
    //  jl = 0 and jl = n means x is out of range
    //
    //  xx is assumed to be monotone increasing
    if (v < k_data[0]) {
      j = 0;
    } else if (v > k_data[k_size_dim0 - 1]) {
      j = k_size_dim0;
    } else {
      j = 1;
      ju = k_size_dim0;
      while (ju - j > 1) {
        jm = static_cast<int32_T>(floor(static_cast<real_T>(ju + j) / 2.0));
        if (v >= k_data[jm - 1]) {
          j = jm;
        } else {
          ju = jm;
        }
      }
    }
    j = static_cast<int32_T>(fmax(
        fmin(static_cast<real_T>(j), static_cast<real_T>(k_size_dim0) - 1.0),
        1.0));
    //  CRRA utility + continuation value
    fx = -(
        -pow(wealth - v, -1.0) +
        (ev_data[(j + ev_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)) - 1] +
         (v - k_data[j - 1]) *
             ((ev_data[j + ev_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)] -
               ev_data[(j +
                        ev_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)) -
                       1]) /
              (k_data[j] - k_data[j - 1]))));
    //  negative because brent_min minimizes
    funccount = 1;
    fv = fx;
    fw = fx;
    xm = 0.5 * (*k_min + kp_ub);
    tol1 = 1.4901161193847656E-8 * fabs(v) + 3.3333333333333334E-9;
    tol2 = 2.0 * tol1;
    //  Main loop
    exitg1 = false;
    while ((static_cast<boolean_T>(!static_cast<int32_T>(exitg1))) &&
           (fabs(xf - xm) > tol2 - 0.5 * (kp_ub - a))) {
      real_T fu;
      real_T x;
      boolean_T guard1;
      //  flag for golden-section step
      //  Attempt parabolic step if possible
      guard1 = false;
      if (fabs(e) > tol1) {
        real_T r;
        r = (xf - w) * (fx - fv);
        x = (xf - v) * (fx - fw);
        fu = (xf - v) * x - (xf - w) * r;
        x = 2.0 * (x - r);
        if (x > 0.0) {
          fu = -fu;
        }
        x = fabs(x);
        r = e;
        e = d;
        //  Check whether the parabolic step is acceptable
        if ((fabs(fu) < fabs(0.5 * x * r)) && (fu > x * (a - xf)) &&
            (fu < x * (kp_ub - xf))) {
          //  Parabolic interpolation step
          d = fu / x;
          x = xf + d;
          //  f must not be evaluated too close to the interval endpoints
          if ((x - a < tol2) || (kp_ub - x < tol2)) {
            if (xm >= xf) {
              d = tol1;
            } else {
              d = -tol1;
            }
          }
        } else {
          //  Reject parabola, do golden-section step
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        //  Golden-section step
        if (xf >= xm) {
          e = a - xf;
        } else {
          e = kp_ub - xf;
        }
        d = 0.3819660112501051 * e;
      }
      //  The function must not be evaluated too close to xf
      if (d >= 0.0) {
        x = xf + fmax(d, tol1);
      } else {
        x = xf + fmin(d, -tol1);
      }
      //  function rbc_solver_interp_gpu
      //  -------------------------------------------------------------------------
      //  RHS of the Bellman equation for given k' (used inside Brent).
      //  This calls user-written function interp1_scal for interpolation.
      //
      //  Returns negative of the objective, since brent_min performs
      //  minimization. Consumption implied by k' Interpolate EV_z at k' Fast
      //  linear interpolation routine Usage: yi = interp1_scal(x,y,xi) where x
      //  and y are column vectors with n elements, xi is a scalar and yi is a
      //  scalar Input Arguments x - Sample points
      //    column vector
      //  Y - Sample data
      //    column vector
      //  xi - Query point
      //    scalar
      // function jl = locate(xx,x)
      //
      //  x is between xx(jl) and xx(jl+1)
      //
      //  jl = 0 and jl = n means x is out of range
      //
      //  xx is assumed to be monotone increasing
      if (x < k_data[0]) {
        j = 0;
      } else if (x > k_data[k_size_dim0 - 1]) {
        j = k_size_dim0;
      } else {
        j = 1;
        ju = k_size_dim0;
        while (ju - j > 1) {
          jm = static_cast<int32_T>(floor(static_cast<real_T>(ju + j) / 2.0));
          if (x >= k_data[jm - 1]) {
            j = jm;
          } else {
            ju = jm;
          }
        }
      }
      j = static_cast<int32_T>(fmax(
          fmin(static_cast<real_T>(j), static_cast<real_T>(k_size_dim0) - 1.0),
          1.0));
      //  CRRA utility + continuation value
      fu = -(-pow(wealth - x, -1.0) +
             (ev_data[(j + ev_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)) -
                      1] +
              (x - k_data[j - 1]) *
                  ((ev_data[j + ev_size_dim0 *
                                    (static_cast<int32_T>(y + 1.0) - 1)] -
                    ev_data[(j + ev_size_dim0 *
                                     (static_cast<int32_T>(y + 1.0) - 1)) -
                            1]) /
                   (k_data[j] - k_data[j - 1]))));
      //  negative because brent_min minimizes
      funccount++;
      b_idx++;
      //  Update a, b, v, w, x, xm, tol1, tol2
      if (fu <= fx) {
        if (x >= xf) {
          a = xf;
        } else {
          kp_ub = xf;
        }
        v = w;
        fv = fw;
        w = xf;
        fw = fx;
        xf = x;
        fx = fu;
      } else {
        if (x < xf) {
          a = x;
        } else {
          kp_ub = x;
        }
        if ((fu <= fw) || (w == xf)) {
          v = w;
          fv = fw;
          w = x;
          fw = fu;
        } else if ((fu <= fv) || (v == xf) || (v == w)) {
          v = x;
          fv = fu;
        }
      }
      xm = 0.5 * (a + kp_ub);
      tol1 = 1.4901161193847656E-8 * fabs(xf) + 3.3333333333333334E-9;
      tol2 = 2.0 * tol1;
      //  Stopping due to limits
      if ((funccount >= 500) || (b_idx >= 500)) {
        exitg1 = true;
      }
    }
    //  Remember: rhs_bellman returns the negative of the value
    v0_data[(static_cast<int32_T>(ik) +
             v0_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)) -
            1] = -fx;
    pol_kp_data[(static_cast<int32_T>(ik) +
                 pol_kp_size_dim0 * (static_cast<int32_T>(y + 1.0) - 1)) -
                1] = xf;
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel6(
    const real_T v0_data[200000], const real_T v_data[], const int32_T b,
    real_T x_data[200000])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<uint64_T>(b);
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    int32_T jm;
    jm = static_cast<int32_T>(idx);
    x_data[jm] = v_data[jm] - v0_data[jm];
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel7(
    const real_T x_data[200000], const int32_T b, real_T y_data[200000])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<uint64_T>(b);
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    int32_T b_idx;
    b_idx = static_cast<int32_T>(idx);
    y_data[b_idx] = fabs(x_data[b_idx]);
  }
}

static __global__ __launch_bounds__(1024, 1) void rbc_solver_interp_gpu_kernel8(
    const real_T v0_data[200000], const int32_T v0_size, real_T v_data[])
{
  uint64_T gStride;
  uint64_T gThreadId;
  uint64_T loopEnd;
  gThreadId = mwGetGlobalThreadIndex();
  gStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<uint64_T>(v0_size);
  for (uint64_T idx{gThreadId}; idx <= loopEnd; idx += gStride) {
    int32_T jm;
    jm = static_cast<int32_T>(idx);
    v_data[jm] = v0_data[jm];
  }
}

void rbc_solver_interp_gpu(rbc_solver_interp_gpuStackData *SD,
                           const real_T cpu_c0_data[], const int32_T c0_size[2],
                           const real_T cpu_k_data[], const int32_T k_size[1],
                           const real_T cpu_pdfz_data[],
                           const int32_T pdfz_size[2], real_T tol,
                           real_T cpu_v_data[], int32_T v_size[2],
                           real_T cpu_pol_kp_data[], int32_T pol_kp_size[2])
{
  dim3 block;
  dim3 grid;
  real_T(*gpu_ev_data)[200000];
  real_T(*gpu_v0_data)[200000];
  real_T(*gpu_x_data)[200000];
  real_T(*gpu_y_data)[200000];
  real_T(*gpu_B_data)[10000];
  real_T diff;
  real_T k_max;
  real_T *gpu_c0_data;
  real_T *gpu_k_data;
  real_T *gpu_k_min;
  real_T *gpu_pdfz_data;
  real_T *gpu_pol_kp_data;
  real_T *gpu_v_data;
  int32_T v0_size[2];
  int32_T x_size[1];
  int32_T its;
  int32_T nk;
  boolean_T c0_data_outdatedOnGpu;
  boolean_T pdfz_data_outdatedOnGpu;
  boolean_T pol_kp_data_outdatedOnCpu;
  boolean_T v_data_outdatedOnCpu;
  boolean_T x_data_outdatedOnCpu;
  checkCudaError(mwCudaMalloc(&gpu_x_data, 1600000ULL), __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_ev_data, 1600000ULL), __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_y_data, 1600000ULL), __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_B_data, 80000ULL), __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_v0_data, 1600000ULL), __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_k_min, 8ULL), __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_pol_kp_data, 200000U * sizeof(real_T)),
                 __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_v_data, 200000U * sizeof(real_T)), __FILE__,
                 __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_pdfz_data, 10000U * sizeof(real_T)),
                 __FILE__, __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_k_data, 2000U * sizeof(real_T)), __FILE__,
                 __LINE__);
  checkCudaError(mwCudaMalloc(&gpu_c0_data, 200000U * sizeof(real_T)), __FILE__,
                 __LINE__);
  x_data_outdatedOnCpu = false;
  pdfz_data_outdatedOnGpu = true;
  c0_data_outdatedOnGpu = true;
  //  Solve the stochastic growth model with VFI and linear interpolation,
  //  using Brent minimization for the choice of next-period capital k'.
  //
  //  Entry point for GPU MEX (GPU Coder).
  //  Parameters
  //  Maximum number of iterations
  //  Discount factor
  //  Risk aversion parameter
  //  Dimensions and grids
  nk = c0_size[0];
  checkCudaError(cudaMemcpy(gpu_k_data, cpu_k_data,
                            static_cast<uint32_T>(k_size[0]) * sizeof(real_T),
                            cudaMemcpyHostToDevice),
                 __FILE__, __LINE__);
  rbc_solver_interp_gpu_kernel1<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(
      gpu_k_data, gpu_k_min);
  k_max = cpu_k_data[c0_size[0] - 1];
  //  Preallocate value and policy functions
  v_data_outdatedOnCpu = false;
  v_size[0] = c0_size[0];
  v_size[1] = c0_size[1];
  //  value function
  v0_size[0] = c0_size[0];
  v0_size[1] = c0_size[1];
  //  previous iterate
  pol_kp_data_outdatedOnCpu = false;
  pol_kp_size[0] = c0_size[0];
  pol_kp_size[1] = c0_size[1];
  //  policy for k'
  //  Value function iteration
  diff = 1.0;
  its = 1;
  while ((diff > tol) && (its < 2000)) {
    uint64_T numIters;
    int32_T nx;
    int32_T y_size_idx_0;
    boolean_T validLaunchParams;
    //  Expected discounted continuation value:
    //  ev(k,z) = beta * sum_{z'} v(k,z') * pdfz(z,z')
    //  Note: pdfz' has size (nz x nz), so v * pdfz' is (nk x nz)
    numIters = computeNumIters(pdfz_size[0] - 1, pdfz_size[1] - 1);
    mwGetLaunchParameters1D(numIters, &grid, &block, 2147483647U);
    if (pdfz_data_outdatedOnGpu) {
      checkCudaError(
          cudaMemcpy(gpu_pdfz_data, cpu_pdfz_data,
                     static_cast<uint32_T>(pdfz_size[0] * pdfz_size[1]) *
                         sizeof(real_T),
                     cudaMemcpyHostToDevice),
          __FILE__, __LINE__);
    }
    pdfz_data_outdatedOnGpu = false;
    validLaunchParams = mwValidateLaunchParameters(grid, block);
    if (validLaunchParams) {
      rbc_solver_interp_gpu_kernel2<<<grid, block>>>(
          gpu_pdfz_data, pdfz_size[0], pdfz_size[1], pdfz_size[1] - 1,
          pdfz_size[0] - 1, *gpu_B_data);
    }
    if ((v_size[1] == 0) || (pdfz_size[1] == 0) || (pdfz_size[0] == 0)) {
      int32_T b_v_size;
      y_size_idx_0 = v_size[0];
      nx = pdfz_size[0];
      b_v_size = v_size[0] * pdfz_size[0] - 1;
      mwGetLaunchParameters1D(computeNumIters(b_v_size), &grid, &block,
                              2147483647U);
      validLaunchParams = mwValidateLaunchParameters(grid, block);
      if (validLaunchParams) {
        rbc_solver_interp_gpu_kernel3<<<grid, block>>>(b_v_size, *gpu_y_data);
      }
    } else {
      real_T beta1;
      y_size_idx_0 = v_size[0];
      nx = pdfz_size[0];
      diff = 1.0;
      beta1 = 0.0;
      cublasDgemm(getCublasGlobalHandle(), CUBLAS_OP_N, CUBLAS_OP_N, v_size[0],
                  pdfz_size[0], v_size[1], (double *)&diff,
                  (double *)&gpu_v_data[0], v_size[0],
                  (double *)&(*gpu_B_data)[0], v_size[1], (double *)&beta1,
                  (double *)&(*gpu_y_data)[0], v_size[0]);
    }
    nx = y_size_idx_0 * nx - 1;
    mwGetLaunchParameters1D(computeNumIters(nx), &grid, &block, 2147483647U);
    validLaunchParams = mwValidateLaunchParameters(grid, block);
    if (validLaunchParams) {
      rbc_solver_interp_gpu_kernel4<<<grid, block>>>(*gpu_y_data, nx,
                                                     *gpu_ev_data);
    }
    //  Flattened loop over all (k,z) pairs: idx = 1, ..., nk*nz
    //  heuristic number of iterations
    mwGetLaunchParameters1D(computeNumIters(nk * c0_size[1] - 1), &grid, &block,
                            2147483647U);
    if (c0_data_outdatedOnGpu) {
      checkCudaError(cudaMemcpy(gpu_c0_data, cpu_c0_data,
                                static_cast<uint32_T>(c0_size[0] * c0_size[1]) *
                                    sizeof(real_T),
                                cudaMemcpyHostToDevice),
                     __FILE__, __LINE__);
    }
    c0_data_outdatedOnGpu = false;
    validLaunchParams = mwValidateLaunchParameters(grid, block);
    if (validLaunchParams) {
      rbc_solver_interp_gpu_kernel5<<<grid, block>>>(
          pol_kp_size[0], v0_size[0], *gpu_ev_data, y_size_idx_0, k_size[0],
          gpu_k_data, gpu_k_min, k_max, gpu_c0_data, c0_size[0], nk,
          nk * c0_size[1] - 1, gpu_pol_kp_data, *gpu_v0_data);
    }
    pol_kp_data_outdatedOnCpu = true;
    //  for idx
    //  Check convergence (max norm)
    if (v_size[0] * v_size[1] == v0_size[0] * v0_size[1]) {
      nx = v_size[0] * v_size[1];
      x_size[0] = nx;
      mwGetLaunchParameters1D(computeNumIters(nx - 1), &grid, &block,
                              2147483647U);
      validLaunchParams = mwValidateLaunchParameters(grid, block);
      if (validLaunchParams) {
        rbc_solver_interp_gpu_kernel6<<<grid, block>>>(*gpu_v0_data, gpu_v_data,
                                                       nx - 1, *gpu_x_data);
      }
      v_data_outdatedOnCpu = false;
      x_data_outdatedOnCpu = true;
    } else {
      if (x_data_outdatedOnCpu) {
        checkCudaError(cudaMemcpy(SD->f0.cpu_x_data, *gpu_x_data, 1600000ULL,
                                  cudaMemcpyDeviceToHost),
                       __FILE__, __LINE__);
      }
      if (v_data_outdatedOnCpu) {
        checkCudaError(cudaMemcpy(cpu_v_data, gpu_v_data,
                                  static_cast<uint32_T>(v_size[0] * v_size[1]) *
                                      sizeof(real_T),
                                  cudaMemcpyDeviceToHost),
                       __FILE__, __LINE__);
      }
      checkCudaError(cudaMemcpy(SD->f0.cpu_v0_data, *gpu_v0_data, 1600000ULL,
                                cudaMemcpyDeviceToHost),
                     __FILE__, __LINE__);
      binary_expand_op(SD->f0.cpu_x_data, x_size, cpu_v_data, v_size,
                       SD->f0.cpu_v0_data, v0_size);
      x_data_outdatedOnCpu = false;
      v_data_outdatedOnCpu = true;
    }
    y_size_idx_0 = x_size[0];
    mwGetLaunchParameters1D(computeNumIters(x_size[0] - 1), &grid, &block,
                            2147483647U);
    if (v_data_outdatedOnCpu) {
      checkCudaError(cudaMemcpy(*gpu_x_data, SD->f0.cpu_x_data, 1600000ULL,
                                cudaMemcpyHostToDevice),
                     __FILE__, __LINE__);
    }
    validLaunchParams = mwValidateLaunchParameters(grid, block);
    if (validLaunchParams) {
      rbc_solver_interp_gpu_kernel7<<<grid, block>>>(*gpu_x_data, x_size[0] - 1,
                                                     *gpu_y_data);
    }
    checkCudaError(cudaMemcpy(SD->f0.cpu_y_data, *gpu_y_data, 1600000ULL,
                              cudaMemcpyDeviceToHost),
                   __FILE__, __LINE__);
    diff = SD->f0.cpu_y_data[0];
    for (nx = 0; nx <= y_size_idx_0 - 2; nx++) {
      if (std::isnan(SD->f0.cpu_y_data[nx + 1])) {
        v_data_outdatedOnCpu = false;
      } else if (std::isnan(diff)) {
        v_data_outdatedOnCpu = true;
      } else {
        v_data_outdatedOnCpu = (diff < SD->f0.cpu_y_data[nx + 1]);
      }
      if (v_data_outdatedOnCpu) {
        diff = SD->f0.cpu_y_data[nx + 1];
      }
    }
    //  Progress print only in normal MATLAB (not in generated MEX)
    //  Update value function
    v_size[0] = v0_size[0];
    v_size[1] = v0_size[1];
    nx = v0_size[0] * v0_size[1] - 1;
    mwGetLaunchParameters1D(computeNumIters(nx), &grid, &block, 2147483647U);
    validLaunchParams = mwValidateLaunchParameters(grid, block);
    if (validLaunchParams) {
      rbc_solver_interp_gpu_kernel8<<<grid, block>>>(*gpu_v0_data, nx,
                                                     gpu_v_data);
    }
    v_data_outdatedOnCpu = true;
    its++;
  }
  //  while
  if (v_data_outdatedOnCpu) {
    checkCudaError(cudaMemcpy(cpu_v_data, gpu_v_data,
                              static_cast<uint32_T>(v_size[0] * v_size[1]) *
                                  sizeof(real_T),
                              cudaMemcpyDeviceToHost),
                   __FILE__, __LINE__);
  }
  if (pol_kp_data_outdatedOnCpu) {
    checkCudaError(cudaMemcpy(cpu_pol_kp_data, gpu_pol_kp_data,
                              static_cast<uint32_T>(c0_size[0] * c0_size[1]) *
                                  sizeof(real_T),
                              cudaMemcpyDeviceToHost),
                   __FILE__, __LINE__);
  }
  checkCudaError(mwCudaFree(gpu_c0_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(gpu_k_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(gpu_pdfz_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(gpu_v_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(gpu_pol_kp_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(gpu_k_min), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(*gpu_v0_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(*gpu_B_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(*gpu_y_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(*gpu_ev_data), __FILE__, __LINE__);
  checkCudaError(mwCudaFree(*gpu_x_data), __FILE__, __LINE__);
}

// End of code generation (rbc_solver_interp_gpu.cu)
