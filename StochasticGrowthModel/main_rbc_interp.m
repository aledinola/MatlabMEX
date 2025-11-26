%% Solve stochastic growth model with linear interp and Brent maximization
% See subfolder "StochasticGrowth_grid" for pure discretization method
% where choice variable k' is restricted on the grid
clear,clc,close all

format compact

%% Numerical parameters
do_compile = 0;
do_compile_gpu = 0;
nz = 25;    % Grid size 
nk = 300;  % grid points + 1

%% Initialize the parameters
beta  = 0.95;   % Discount rate
eta   = 2;      % Risk aversion parameter
alpha = 0.33;   % Technology parameter
delta = 0.1;    % Depreciation rate
rho   = 0.90;   % Tech. shock persistence
sigma = 0.01;   % Tech. shock st. dev.
Tsim  = 10000;  % Num. periods for simulation

%% Grid for productivity z
muz   = 0;
width = 4.2; 

[z, pdfz] = tauchen(nz,muz,rho,sigma,width);
z = exp(z);
temp = pdfz^1000;
pdfz_star = temp(1,:); % Stationary distribution of Markov chain for z
pdfz_star = pdfz_star/sum(pdfz_star);

%% Grid for capital k
kstar = (alpha/(1/beta - (1-delta)))^(1/(1-alpha)); % steady state k
cstar = kstar^(alpha) - delta*kstar;
istar = delta*kstar;
ystar = kstar^(alpha);

kmin = 0.5*kstar;
kmax = 1.5*kstar;

k      = linspace(kmin,kmax,nk)'; % k today is (nk,1)
kpgrid = k'; % k' tomorrow is (1,nk)

c0 = zeros(nk,nz);  % total wealth
for iz=1:nz
    c0(:,iz) = z(iz)*k.^alpha + (1-delta)*k;
end

%% run solver in native Matlab 
disp('MATLAB NATIVE (with parfor)')
tic;  % <----- Start the timer
[v, pol_kp_val] = rbc_solver_interp(c0,k,pdfz);
time_v = toc;

%% Mex on cpu

if do_compile==1
    disp('Compiling MEX on CPU...')
    cfg = coder.config('mex');
    cfg.GenerateReport = true;
    codegen -config cfg rbc_solver_interp -args {c0,k,pdfz } -o rbc_solver_interp_mex
    disp('Compilation done!')
end

disp('MATLAB MEX on CPU')
tic;  % <----- Start the timer
[v2, pol_kp_val2] = rbc_solver_interp_mex(c0,k,pdfz);
time_mex = toc;

%% Mex on GPU

if do_compile_gpu==1
    reset(gpuDevice)
    disp('Compiling MEX on GPU...')
    cfg = coder.gpuConfig('mex');
    cfg.GenerateReport = true;
    codegen -config cfg rbc_solver_interp_gpu -args {c0,k,pdfz } -o rbc_solver_interp_gpu_mex
    disp('Compilation done!')
end

disp('MATLAB MEX on GPU-CUDA')
tic;  % <----- Start the timer
[v3, pol_kp_val3] = rbc_solver_interp_gpu_mex(c0,k,pdfz);
time_mex_gpu = toc;

err_mex = max(abs(pol_kp_val-pol_kp_val2),[],"all");
err_mex_gpu = max(abs(pol_kp_val-pol_kp_val3),[],"all");

disp('PARAMETER VALUES')
fprintf('nk (Num. grid points for endog. state: %d \n',nk)
fprintf('nz (Num. grid points for exog. state : %d \n',nz)
disp('RESULTS')
fprintf('Max abs diff Matlab native vs MEX:     %f \n',err_mex)
fprintf('Max abs diff Matlab native vs MEX GPU: %f \n',err_mex_gpu)
fprintf('Time for VFI, matlab:  %f \n',time_v)
fprintf('Time for VFI, mex:     %f \n',time_mex)
fprintf('Time for VFI, mex GPU: %f \n',time_mex_gpu)


% %% Simulation with interpolation
% 
% tic
% 
% sim_k = zeros(Tsim+1,1);
% sim_c = zeros(Tsim,1);
% k0 = kstar;
% z0 = floor((nz+1)/2);
% 
% % --- Step 1: Simulate exogenous state variable and get sequence {z(t)}
% sim_z_ind = markov_simul(pdfz,Tsim,z0,1);
% sim_z = z(sim_z_ind);
% 
% % --- Step 2: Given {z(t)}, simulate {k(t)} from t=1 to t=T as follows:
% % k(0)>0 is given (initial condition)
% % k(1) = policy(k(0),z(0))
% % k(2) = policy(k(1),z(1))
% % ...
% % k(T) = policy(k(T-1),z(T-1))
% sim_k(1) = k0;
% for t=1:Tsim
%     sim_k(t+1) = interp1_scal(k,pol_kp_val(:,sim_z_ind(t)),sim_k(t));
%     sim_c(t) = sim_z(t)*sim_k(t)^alpha + (1-delta)*sim_k(t) - sim_k(t+1);
% end
% time_s = toc;

%% Results


%fprintf('Time for Simulation (seconds): %f \n',time_s)

figure
plot(k,v,"LineWidth",2)
xlabel('Capital, k')
title('Value function V(k,z)')
print('V','-dpng')

figure
plot(k,pol_kp_val,"LineWidth",2)
xlabel('Capital, k')
title('Policy function k''(k,z)')
print('pol_kp','-dpng')

figure
plot(k,k,'--',"LineWidth",2)
hold on
plot(k,pol_kp_val(:,1),"LineWidth",2)
hold on
plot(k,pol_kp_val(:,nz),"LineWidth",2)
xlabel('Capital, k')
title('Policy function k''(k,z)')
print('pol_kp2','-dpng')

% figure
% plot(sim_z(1:1000),"LineWidth",2)
% hold on
% plot(sim_k(1:1000),"LineWidth",2)
% hold on
% plot(sim_c(1:1000),"LineWidth",2)
% legend('TFP','Capital','Consumption')
% xlabel('Time periods')
% title('Simulation')
% print('simul','-dpng')



