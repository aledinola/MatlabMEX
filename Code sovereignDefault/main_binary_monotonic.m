clear
% Section 1 uses bruteforce
% Section 2 uses binary monotonicity

format compact

%%
nb = 200;   % change number of nb within vector [200, 400]

run_bruteforce_matlab = 0 ;  % 0 for false, 1 for true

compile_bruteforce = 0 ;
run_bruteforce_mex = 1 ;

run_binarymonotonic_matlab = 0 ;

compile_binarymonotonic = 0 ;
run_binarymonotonic_mex = 1 ;

%% define parameters
para.phi0 = 0.028  ; 
para.rstar = 0.01 ;  %quarterly risk-free interest rate (Chatterjee and Eyigungor, AER 2012). 
para.theta = 0.0385 ; %probability of reentyy (USG  and Chatterjee and Eyigungor, AER 2012). 
para.sigg = 2; %intertemporal elasticity of consumption substitution 
para.betta = 0.85 ;  %discount factor
nz = 25 ; % number of transitory shock grids
nm = 25 ;

%% preparation
% for transitory shocks
rhoz = 0.90 ; %mean reversion of log prod  %% 0.86 from data
sdz = 0.030 ;  %stdev of log prod shock 
width = 4.0 ;  % 3.7 (two-tail) sigma both sides covers 99.98% of the distribution
muz = 0; %long run mean of log prod
% for trend shocks
rhom = 0.90 ; %mean reversion of log prod  %% 0.86 from data
sdm = 0.008 ;  %stdev of log prod shock 
mum = 1.02; %long run mean of log prod

[zgrid,pdfz] = tauchen(nz,muz,rhoz,sdz,width);
[mgrid,pdfm] = tauchen(nm,mum,rhom,sdm,width);

ns = nz*nm;
pdf = kron(pdfm,pdfz); % Joint distribution, 
pdf = pdf ./ (((sum(pdf,2)))*ones(1,ns)); 

z = zeros(ns,1) ;  m = zeros(ns,1) ; 
for im = 1:nm
    for iz = 1:nz 
        is = (im-1)*nz + iz ;
        z(is) = zgrid(iz) ;
        m(is) = mgrid(im) ; 
    end
end

%debt grid
bupper = 1.01;    blower = 0;
b = blower:(bupper-blower)/(nb-1):bupper;
b = b(:);

%%
if run_bruteforce_matlab || run_binarymonotonic_matlab
    if isempty(gcp('nocreate'))
    parpool
    end
end

%% 1. Brute forece 

%% 1.1 Brute force with Matlab

if run_bruteforce_matlab

[q,bp,vp,def,totaltime,avgtime] = solver_bruteforce(b,z,m,pdf,para);
disp(['BruteForce serial time total: ', num2str(totaltime)]) ;
disp(['BruteForce serial time average: ', num2str(avgtime)]) ;


[q,bp,vp,def,totaltime,avgtime] = solver_bf_parfor(b,z,m,pdf,para);
disp(['BruteForce parfor time total: ', num2str(totaltime)]) ;
disp(['BruteForce parfor time average: ', num2str(avgtime)]) ;

end

%% 1.2 Brute force with Mex 
if compile_bruteforce

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_bruteforce -args {b,z,m,pdf,para} -o solver_bruteforce_mex

codegen -config cfg solver_bf_parfor -args {b,z,m,pdf,para } -o solver_bf_parfor_mex

cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_bruteforce_cuda -args {b,z,m,pdf,para } -o solver_bf_cuda_mex

end

% run mex code
if run_bruteforce_mex

[q,bp,vp,def,totaltime,avgtime] = solver_bruteforce_mex(b,z,m,pdf,para);
disp(['BruteForce mex time total: ', num2str(totaltime)]) ;
disp(['BruteForce mex time average: ', num2str(avgtime)]) ;

[q,bp,vp,def,totaltime,avgtime] = solver_bf_parfor_mex(b,z,m,pdf,para);
disp(['BruteForce mex parfor time total: ', num2str(totaltime)]) ;
disp(['BruteForce mex parfor time average: ', num2str(avgtime)]) ;

reset(gpuDevice)
[q1,bp,vp,def,totaltime,avgtime] = solver_bf_cuda_mex(b,z,m,pdf,para);
disp(['BruteForce mex cuda time total: ', num2str(totaltime)]) ;
disp(['BruteForce mex cuda time average: ', num2str(avgtime)]) ;

end

%% 2. Divide and Conquer (binary monotonic)

%% 2.1. binary monotonic with Matlab 

if run_binarymonotonic_matlab

[q_1,bp_1,vp_1,def_1,totaltime,avgtime] = solver_divide(b,z,m,pdf,para);
disp(['Binary serial time total: ', num2str(totaltime)]) ;
disp(['Binary serial time average: ', num2str(avgtime)]) ;

[q_1,bp_1,vp_1,def_1,totaltime,avgtime] = solver_divide_parfor(b,z,m,pdf,para);
disp(['Binary parfor time total: ', num2str(totaltime)]) ;
disp(['Binary parfor time average: ', num2str(avgtime)]) ;

end

%% 2.2. binary monotonic with Mex

if compile_binarymonotonic

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_divide -args {b,z,m,pdf,para } -o solver_divide_mex

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_divide_parfor -args {b,z,m,pdf,para} -o solver_divide_parfor_mex

cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_divide_cuda -args {b,z,m,pdf,para} -o solver_divide_cuda_mex

end

if run_binarymonotonic_mex

[q_11,bp_11,vp_11,def_11,totaltime,avgtime] = solver_divide_mex(b,z,m,pdf,para);
disp(['Binary mex serial time total: ', num2str(totaltime)]) ;
disp(['Binary mex serial time average: ', num2str(avgtime)]) ;

[q_11,bp_11,vp_11,def_11,totaltime,avgtime] = solver_divide_parfor_mex(b,z,m,pdf,para);
disp(['Binary mex parfor time total: ', num2str(totaltime)]) ;
disp(['Binary mex parfor time average: ', num2str(avgtime)]) ;

reset(gpuDevice)
[q_12,bp_12,vp_12,def_12,totaltime,avgtime] = solver_divide_cuda_mex(b,z,m,pdf,para);
disp(['Binary mex cuda time total: ', num2str(totaltime)]) ;
disp(['Binary mex cuda time average: ', num2str(avgtime)]) ;

end


