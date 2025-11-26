%% Solve stochastic growth model with linear interpolation and Brent maximization
clear; clc; close all;
format compact

%% Numerical parameters
do_compile     = 0;  % Compile CPU MEX version (set to 1 if you want to recompile)
do_compile_gpu = 0;  % Compile GPU MEX version
do_run_mex_cpu = 1;
do_run_mex_gpu = 1;

nz  = 25;    % Number of grid points for productivity
nk  = 300;   % Number of grid points for capital
tol = 1e-9;   % Tolerance for value function iteration

% Upper bounds for variable-size arrays used in GPU code generation
max_nk = 2000; % Max number of capital grid points
max_nz = 100;  % Max number of z states

%% Model parameters
beta  = 0.95;   % Discount factor
eta   = 2;      % Risk aversion parameter
alpha = 0.33;   % Capital share in production
delta = 0.10;   % Depreciation rate
rho   = 0.90;   % AR(1) persistence of technology shock
sigma = 0.01;   % Std. dev. of technology shock
Tsim  = 10000;  % Number of periods for simulation

%% Grid for productivity z
muz   = 0.0;
width = 3.0;

[z, pdfz] = tauchen(nz, muz, rho, sigma, width);
z         = exp(z);

% Stationary distribution of the Markov chain for z
temp      = pdfz^1000;
pdfz_star = temp(1, :);
pdfz_star = pdfz_star / sum(pdfz_star);

%% Steady state and grid for capital k
kstar = (alpha / (1 / beta - (1 - delta)))^(1 / (1 - alpha)); % Steady-state k
cstar = kstar^alpha - delta * kstar;  % Steady-state consumption (unused, kept for reference)
istar = delta * kstar;                % Steady-state investment
ystar = kstar^alpha;                  % Steady-state output

kmin = 0.5 * kstar;
kmax = 1.5 * kstar;

k      = linspace(kmin, kmax, nk)'; % k today: (nk x 1)

% Total resources available (z * f(k) + (1 - delta) * k)
c0 = zeros(nk, nz);
for iz = 1:nz
    c0(:, iz) = z(iz) * k.^alpha + (1 - delta) * k;
end

%% Compile CPU MEX (optional)
if do_compile == 1
    disp('Compiling MEX on CPU...')
    cfg = coder.config('mex');
    cfg.GenerateReport = true;

    % Variable-size inputs for CPU code generation (unbounded sizes allowed on CPU)
    c0_Coder   = coder.typeof(0, [Inf, Inf]);
    k_Coder    = coder.typeof(0, [Inf, 1]);
    pdfz_Coder = coder.typeof(0, [Inf, Inf]);

    codegen -config cfg rbc_solver_interp ...
        -args {c0_Coder, k_Coder, pdfz_Coder, tol} ...
        -o rbc_solver_interp_mex;

    disp('CPU MEX compilation done!')
end

%% Compile GPU MEX (CUDA)
if do_compile_gpu == 1
    reset(gpuDevice)
    disp('Compiling MEX on GPU...')
    cfg = coder.gpuConfig('mex');
    cfg.GenerateReport = true;

    % Variable-size inputs with maximum sizes for GPU code generation
    % Third argument [true/false] indicates which dimensions are variable
    c0_Coder_gpu   = coder.typeof(0, [max_nk, max_nz], [true, true]);  % c0: nk x nz
    k_Coder_gpu    = coder.typeof(0, [max_nk, 1],      [true, false]); % k:  nk x 1
    pdfz_Coder_gpu = coder.typeof(0, [max_nz, max_nz], [true, true]);  % pdfz: nz x nz


    codegen -config cfg rbc_solver_interp_gpu ...
        -args {c0_Coder_gpu, k_Coder_gpu, pdfz_Coder_gpu, tol} ...
        -o rbc_solver_interp_gpu_mex;

    disp('GPU MEX compilation done!')
end

%% Run solver in native MATLAB
disp('MATLAB NATIVE (with parfor inside the solver, if implemented)')
tic;
[v, pol_kp_val] = rbc_solver_interp(c0, k, pdfz, tol);
time_v = toc;

%% Run CPU MEX (assuming it is already compiled / available on path)
if do_run_mex_cpu==1
    disp('MATLAB MEX on CPU')
    tic;
    [v2, pol_kp_val2] = rbc_solver_interp_mex(c0, k, pdfz, tol);
    time_mex = toc;
end

%% Run GPU MEX
if do_run_mex_gpu==1
    disp('MATLAB MEX on GPU-CUDA')
    tic;
    [v3, pol_kp_val3] = rbc_solver_interp_gpu_mex(c0, k, pdfz, tol);
    time_mex_gpu = toc;
end

%% Accuracy and timing comparisons
if do_run_mex_cpu==1
    err_mex     = max(abs(pol_kp_val  - pol_kp_val2), [], "all");
end
if do_run_mex_gpu==1
    err_mex_gpu = max(abs(pol_kp_val  - pol_kp_val3), [], "all");
    err_mex_gpu_v = max(abs(v  - v3), [], "all");
end

disp(' ')
disp('PARAMETER VALUES')
fprintf('nk (num. grid points for endogenous state k): %d\n', nk);
fprintf('nz (num. grid points for exogenous state  z): %d\n', nz);

disp(' ')
disp('RESULTS')

if do_run_mex_cpu==1 && do_run_mex_gpu==1
    fprintf('Max abs diff: MATLAB native vs CPU MEX     = %e\n', err_mex);
    fprintf('Max abs diff: MATLAB native vs GPU MEX     = %e\n', err_mex_gpu);
    fprintf('Time for VFI, MATLAB native: %8.4f seconds\n', time_v);
    fprintf('Time for VFI, CPU MEX:      %8.4f seconds\n', time_mex);
    fprintf('Time for VFI, GPU MEX:      %8.4f seconds\n', time_mex_gpu);
elseif do_run_mex_cpu==1
    fprintf('Max abs diff: MATLAB native vs CPU MEX     = %e\n', err_mex);
    fprintf('Time for VFI, MATLAB native: %8.4f seconds\n', time_v);
    fprintf('Time for VFI, CPU MEX:      %8.4f seconds\n', time_mex);
elseif do_run_mex_gpu==1
    fprintf('Max abs diff: MATLAB native vs GPU MEX     = %e\n', err_mex_gpu);
    fprintf('Time for VFI, MATLAB native: %8.4f seconds\n', time_v);
    fprintf('Time for VFI, GPU MEX:      %8.4f seconds\n', time_mex_gpu);
end

%% Plots: value and policy functions

if do_run_mex_cpu==1
    figure
    plot(k, v2, "LineWidth", 2)
    xlabel('Capital, k')
    ylabel('Value, V(k,z)')
    title('Value function V(k,z)')
    print('V', '-dpng')

    figure
    plot(k, k, '--', "LineWidth", 2); hold on
    plot(k, pol_kp_val2(:, 1),  "LineWidth", 2);
    plot(k, pol_kp_val2(:, nz), "LineWidth", 2);
    xlabel('Capital, k')
    ylabel('Next-period capital, k''')
    title('Policy function k''(k,z), MEX on CPU')
    legend({'45-degree line', 'Low z state', 'High z state'}, 'Location', 'NorthWest')
    print('pol_kp2', '-dpng')
end

if do_run_mex_gpu==1
    figure
    plot(k, k, '--', "LineWidth", 2); hold on
    plot(k, pol_kp_val3(:, 1),  "LineWidth", 2);
    plot(k, pol_kp_val3(:, nz), "LineWidth", 2);
    xlabel('Capital, k')
    ylabel('Next-period capital, k''')
    title('Policy function k''(k,z), MEX on GPU')
    legend({'45-degree line', 'Low z state', 'High z state'}, 'Location', 'NorthWest')
    print('pol_kp3', '-dpng')
end