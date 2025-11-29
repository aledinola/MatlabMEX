clear

format compact

run_matlab = 1 ;  % 0 for false, 1 for true
compile_mex = 0 ;
run_mex = 0 ;

nz = 25 ;   % number of transitory shock grids
nm = 25 ;   % number of trend shock grids

%% calibration
para.phi0 = 0.028  ; 
para.rstar = 0.01 ;  
para.theta = 0.0385 ; %probability of reentry
para.sigg = 2; %intertemporal elasticity of consumption substitution 
para.betta = 0.85 ;  %discount factor
nb = 80 ;   % # of grid points for debt 

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
bupper = 1.01 ;    blower = 0;
b = blower:(bupper-blower)/(nb-1):bupper;
b = b(:);

%%
if run_matlab 
    if isempty(gcp('nocreate'))
    parpool
    end
end

%% run solver on Native Matlab 

if run_matlab

[q,bp,vp,def,totaltime,avgtime] = solver_interp_parfor(b,z,m,pdf,para) ;
disp(['Parfor time total: ', num2str(totaltime)]) ;
disp(['Parfor time average: ', num2str(avgtime)]) ;

[q,bp,vp,def,totaltime,avgtime] = solver_interp(b,z,m,pdf,para) ;
disp(['Serial time total: ', num2str(totaltime)]) ;
disp(['Serial time average: ', num2str(avgtime)]) ;

end


%% run solve on Mex

if compile_mex

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_interp -args {b,z,m,pdf,para} -o solver_interp_serial_mex

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_interp_parfor -args {b,z,m,pdf,para} -o solver_interp_parfor_mex

cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_interp_cuda -args {b,z,m,pdf,para} -o solver_interp_cuda_mex

end



if run_mex

% [q,bp,vp,def,totaltime,avgtime] = solver_interp_serial_mex(b,z,m,pdf,para) ;
% disp(['Mex serial time total: ', num2str(totaltime)]) ;
% disp(['Mex serial time average: ', num2str(avgtime)]) ;

% [q,bp,vp,def,totaltime,avgtime] = solver_interp_parfor_mex(b,z,m,pdf,para) ;
% disp(['Mex parfor time total: ', num2str(totaltime)]) ;
% disp(['Mex parfor time average: ', num2str(avgtime)]) ;

reset(gpuDevice)
[q,bp,vp,def,totaltime,avgtime] = solver_interp_cuda_mex(b,z,m,pdf,para) ;
disp(['Mex cuda time total: ', num2str(totaltime)]) ;
disp(['Mex cuda time average: ', num2str(avgtime)]) ;

end


