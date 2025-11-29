clear

format compact

run_matlab = 1 ;  % 0 for false, 1 for true

compile_mex = 0 ;
run_mex = 0 ;

nb = 200 ; % 200 or 400 ;
%%
phi0 = 0.028  ; 
rstar = 0.01 ;  
theta = 0.0385 ; 
sigg = 2; %intertemporal elasticity of consumption substitution 
betta = 0.85 ;  %discount factor

para.phi0 = phi0  ; 
para.rstar = rstar ;  
para.theta = theta ; 
para.sigg = sigg; %intertemporal elasticity of consumption substitution 
para.betta = betta ;  %discount factor

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
[~,nb0] = min(abs(b));  b(nb0) = 0; %force the element closest to zero to be exactly zero

%% matlab native
if run_matlab

[q,bp,vp,def,totaltime,avgtime] = solver_vec(b,z,m,pdf,para) ;
disp(['Matlab vec time total: ', num2str(totaltime)]) ;
disp(['Matlab vec time average: ', num2str(avgtime)]) ;

reset(gpuDevice)
[q1,bp1,vp1,def1,totaltime1,avgtime1] = solver_vec_gpu(b,z,m,pdf,para) ;
disp(['gpuArray time total: ', num2str(totaltime1)]) ;
disp(['gpuArray time average: ', num2str(avgtime1)]) ;

end

%% Matlab native GPU

if compile_mex
cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg solver_vec_cuda -args {b,z,m,pdf,para} -o solver_vec_cuda_mex
end

if run_mex

reset(gpuDevice)
[q,bp,vp,def,totaltime,avgtime] = solver_vec_cuda_mex(b,z,m,pdf,para) ;
disp(['Mex vec cuda time total: ', num2str(totaltime)]) ;
disp(['Mex vec cuda time average: ', num2str(avgtime)]) ;

end




