% RBC_SOLVER_INTERP_GPU_SCRIPT   Generate MEX-function rbc_solver_interp_gpu_mex
%  from rbc_solver_interp_gpu.
% 
% Script generated from project 'rbc_solver_interp_gpu.prj' on 04-Feb-2025.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

clear all
clc

%% Create configuration object of class 'coder.MexCodeConfig'.
cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;

%% Define argument types for entry-point 'rbc_solver_interp_gpu'.
ARGS = cell(1,1);
ARGS{1} = cell(3,1);
ARGS{1}{1} = coder.typeof(0, [1000,1000], [1, 1], 'Gpu', true); % c0(k,z)
ARGS{1}{2} = coder.typeof(0, [1000,1],    [1, 0], 'Gpu', true); % k grid
ARGS{1}{3} = coder.typeof(0, [1000,1000], [1, 1], 'Gpu', true); % pdfz

%% Invoke MATLAB Coder.
codegen -config cfg rbc_solver_interp_gpu -args ARGS{1}

