% Buera & Shin (2013) - Financial Frictions and the Persistence of History: 
% A Quantitative Exploration
clear ; 
clc
close all

format compact

run_compile = 1 ;
run_mex = 1 ;

n_a    = 1501; % Num of grid points, choose n_a = 1001 or 1501

%% Set folders
ResFolder = 'results'; % Folder to save results
InpFolder = 'inputs';  % Folder where model inputs are saved

if ~isfolder(ResFolder)
    warning('Subfolder for results does not exist.. creating it now')
    mkdir(ResFolder)
end
if ~isfolder(InpFolder)
    warning('Subfolder for inputs does not exist..')
end

%% Flags and numerical options
do_GE          = 1; % 0 = partial equilibrium, 1 = general equilibrium
do_replication = 0; % Flag 0/1 to replicate Figure 2 of BS 2013. This 
                    % requires repeatedly solving the s.s. for different
                    % lambdas and it takes some time
% Options for value function iteration:
vfoptions           = struct();
vfoptions.verbose   = 1;
vfoptions.lowmemory = 0;
vfoptions.tolerance = 1e-9;
vfoptions.howards   = 80;
% Options for stationary distribution:
simoptions           = struct();
simoptions.verbose   = 1;
simoptions.tolerance = 1e-6;
simoptions.maxit     = 10000;
% Options for GE loop:
heteroagentoptions.do_GE    = do_GE ;
heteroagentoptions.fminalgo = 1 ; %1=fminsearch, 2=lsqnonlin
heteroagentoptions.maxiter  = 200 ;
heteroagentoptions.verbose  = 1 ;
heteroagentoptions.toleranceGEprices=10^(-4) ;
heteroagentoptions.toleranceGEcondns=10^(-4) ;

%% Parameters

% Preferences
Params.crra = 1.5;   % CRRA utility param
Params.beta = 0.904; % Discount factor

% Production fn
Params.delta   =0.06; % Capital depreciation rate
Params.alpha   =0.33; % Capital coefficient
Params.upsilon =0.21; % 1 minus span of control parameter

% Entrepreneurial ability shocks
Params.psi     =0.894; %stochastic process: persistence
Params.eta     =4.15;  %stochastic process: dispersion

% Collateral constraint
Params.lambda  = inf; % Calibration of steady-state done for US economy

% Initial values for general eqm parameters: good for lambda=inf
Params.r = 0.0476;  % 0.0476;
Params.w = 0.172;   % 0.172;

%% Grid for assets
% grid on assets

a_min  = 1e-6; % Lower bound
a_max  = 4000; % Upper bound
% a_scale>1 puts more points near zero
a_scale = 2;   % "Curvature" of asset grid
a_grid  = a_min+(a_max-a_min)*linspace(0,1,n_a)'.^a_scale;

%% Stochastic process
z_grid   = importdata(fullfile(InpFolder,'support.dat')) ;
pi_z_vec = importdata(fullfile(InpFolder,'dist.dat')) ;
n_z      = length(z_grid); % Num of grid points for exo state z
% See my notes on Buera and Shin paper
pi_z = Params.psi*eye(n_z)+(1-Params.psi)*ones(n_z,1)*pi_z_vec' ;
pi_z = pi_z./sum(pi_z,2);

%% Compute the model once, either in partial or general equilibrium
GEPrices0(1) = Params.r ;
GEPrices0(2) = Params.w ;
if do_GE==1
    disp('Solving initial stationary general eqm')
else
    disp('Solving initial partial eqm')
end

%% Compile Mex 

if run_compile

vs_a_grid = coder.typeof(a_grid,[3001,1],1) ;

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg fun_VFI_parfor2 -args {zeros(1,2), vs_a_grid, z_grid, pi_z, Params, vfoptions} -o fun_VFI_parforVS_mex

cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg fun_VFI_cuda -args {zeros(1,2), vs_a_grid, z_grid, pi_z, Params, vfoptions} -o fun_VFI_cudaVS_mex

cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg fun_return_cuda -args {zeros(1,2), vs_a_grid, z_grid, Params, vfoptions} -o fun_return_cudaVS_mex

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg fun_VFIonly_parfor -args {vs_a_grid, z_grid, pi_z, Params, vfoptions, vs_return, zeros(1,1)} -o fun_VFIonly_parforVS_mex

end

%% run Mex 

if run_mex

%% run Mex parfor 

smctime = tic;
if heteroagentoptions.do_GE==1
    % Find GE prices
    if heteroagentoptions.fminalgo==1
        % fminsearch
        minoptions = optimset('TolX',heteroagentoptions.toleranceGEprices,'TolFun',heteroagentoptions.toleranceGEcondns,'MaxFunEvals',heteroagentoptions.maxiter);
        p_eqm = fminsearch(@(p) model_subfuns_parfor_mexVS(p, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions),GEPrices0,minoptions);

    elseif heteroagentoptions.fminalgo==2
        % Matlab lsqnonlin()
        minoptions = optimoptions('lsqnonlin','FiniteDifferenceStepSize',1e-2,'TolX',heteroagentoptions.toleranceGEprices,'TolFun',heteroagentoptions.toleranceGEcondns,'MaxFunEvals',heteroagentoptions.maxiter,'MaxIter',heteroagentoptions.maxiter);
        p_eqm = lsqnonlin(@(p) model_subfuns_parfor_mexVS(p, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions),GEPrices0,[],[],[],[],[],[],[],minoptions);
    else
        error('Selected fminalgo does not exist')
    end %end if

    [~,V,Policy,StatDist] = model_subfuns_parfor_mexVS(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions); % Recompute model objects at GE prices
elseif heteroagentoptions.do_GE==0
    p_eqm = GEPrices0;
    [~,V,Policy,StatDist] = model_subfuns_parfor_mexVS(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions);
end

Outputs_parfor_mex = compute_targets(a_grid, z_grid, pi_z, Policy, StatDist, Params, p_eqm) ;
totaltime_parfor_mex = toc(smctime)
make_table(ResFolder,Outputs_parfor_mex,Params) ;

%% run Mex cuda

reset(gpuDevice)
smctime = tic;
if heteroagentoptions.do_GE==1
    % Find GE prices
    if heteroagentoptions.fminalgo==1
        % fminsearch
        minoptions = optimset('TolX',heteroagentoptions.toleranceGEprices,'TolFun',heteroagentoptions.toleranceGEcondns,'MaxFunEvals',heteroagentoptions.maxiter);
        p_eqm = fminsearch(@(p) model_subfuns_cudaVS(p, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions),GEPrices0,minoptions);

    elseif heteroagentoptions.fminalgo==2
        % Matlab lsqnonlin()
        minoptions = optimoptions('lsqnonlin','FiniteDifferenceStepSize',1e-2,'TolX',heteroagentoptions.toleranceGEprices,'TolFun',heteroagentoptions.toleranceGEcondns,'MaxFunEvals',heteroagentoptions.maxiter,'MaxIter',heteroagentoptions.maxiter);
        p_eqm = lsqnonlin(@(p) model_subfuns_cudaVS(p, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions),GEPrices0,[],[],[],[],[],[],[],minoptions);
    else
        error('Selected fminalgo does not exist')
    end %end if

    [~,V,Policy,StatDist] = model_subfuns_cudaVS(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions); % Recompute model objects at GE prices
elseif heteroagentoptions.do_GE==0
    p_eqm = GEPrices0;
    [~,V,Policy,StatDist] = model_subfuns_cudaVS(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions);
end

Outputs_cuda = compute_targets(a_grid, z_grid, pi_z, Policy, StatDist, Params, p_eqm) ;
totaltime_cuda = toc(smctime)
make_table(ResFolder,Outputs_cuda,Params) ;


end


%% Replicate Figure 2 of BS2013

if do_replication==1

heteroagentoptions.do_GE=1; % Figure 1 of BS is in general equil, so we enforce this
%ii_bench    = 1;
lambda_vec = [inf,2.0,1.75,1.5,1.25,1.0]';
NN = length(lambda_vec);
do_GE = 1;

%Pre-allocate arrays or structures where you want to store the output
share_entre_vec = zeros(NN,1);
extfin_vec      = zeros(NN,1);
extfin_Y_vec    = zeros(NN,1);
Y_vec           = zeros(NN,1);
r_vec           = zeros(NN,1);
w_vec           = zeros(NN,1);
K_vec           = zeros(NN,1);
K_Y_vec         = zeros(NN,1);
GE_resid_vec    = zeros(NN,1);

for ii=1:length(lambda_vec)

    %Assign lambda:
    Params.lambda = lambda_vec(ii);

    disp('***************************************************************')
    fprintf('Doing experiment %d of %d \n',ii,length(lambda_vec));
    fprintf('lambda = %.3f \n',lambda_vec(ii));
    disp('***************************************************************')

    if ii==1
        Params.r = 0.0472;
        Params.w = 0.171;
    elseif ii>1
        Params.r = r_vec(ii-1) ;
        Params.w = w_vec(ii-1);
    end

    GEPrices0(1) = Params.r;
    GEPrices0(2) = Params.w;

    tic
    [Outputs] = solve_model(GEPrices0,Params,vfoptions,simoptions,heteroagentoptions);
    toc

    %Aggregate quantities and prices
    share_entre_vec(ii) = Outputs.share_entre;
    extfin_vec(ii)      = Outputs.extfin;
    Y_vec(ii)           = Outputs.Y;
    r_vec(ii)           = Outputs.r;
    w_vec(ii)           = Outputs.w;
    K_vec(ii)           = Outputs.K;
    K_Y_vec(ii)         = Outputs.K_Y;
    extfin_Y_vec(ii)    = Outputs.extfin/Outputs.Y;
    GE_resid_vec(ii,:)  = Outputs.GE_eqns_vec;

end

%Add "_norm" to denote change wrt benchmark
ii_bench    = 1;
Y_vec_norm  = zeros(NN,1);

for ii=1:NN
    Y_vec_norm(ii) = (Y_vec(ii)/Y_vec(ii_bench));
end

%% Plots for Figure 2 of the paper

ldw = 2;
fts = 14;

figure
plot(extfin_Y_vec,Y_vec_norm,'linewidth',ldw)
xlabel('External Finance to GDP','FontSize',fts)
ylabel('GDP relative to benchmark','FontSize',fts)
title('GDP and TFP','FontSize',fts)
print(fullfile(ResFolder,'fig2a_BS2013'),'-dpng')

figure
plot(extfin_Y_vec,r_vec,'linewidth',ldw)
xlabel('External Finance to GDP','FontSize',fts)
ylabel('Interest rate','FontSize',fts)
title('Interest Rate','FontSize',fts)
print(fullfile(ResFolder,'fig2b_BS2013'),'-dpng')

figure
subplot(1,2,1)
    plot(extfin_Y_vec,Y_vec_norm,'linewidth',ldw)
    xlabel('External Finance to GDP','FontSize',fts)
    ylabel('GDP relative to benchmark','FontSize',fts)
    title('GDP and TFP','FontSize',fts)
subplot(1,2,2)
    plot(extfin_Y_vec,r_vec,'linewidth',ldw)
    xlabel('External Finance to GDP','FontSize',fts)
    ylabel('Interest rate','FontSize',fts)
    title('Interest Rate','FontSize',fts)
print(fullfile(ResFolder,'fig2_BS2013'),'-dpng')

save (fullfile(ResFolder,"data_all.mat")) 

end %end if do_replication