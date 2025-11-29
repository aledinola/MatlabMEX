clear
format compact

nb = 200 ;    % change number of nb within vector [200, 400]

generate_mexserial = 0 ;    % 0 for false        % Generate CPU MEX code
run_mexserial = 0 ;         % 0 for false            % Run MEX/CUDA compiled solvers
generate_mex = 0 ;          % 0 for false    % Generate CPU MEX code and CUDA MEX code
run_mex = 1 ;

%% Calibration
phi0 = 0.10  ; 
phi1 = -0.37 ; % higher abs(phi1) => the initial first grid of zaut<z is at higher zi 
              %  (smaller z index) so higher default
              % if dy too high, increase abs(phi)
phi2 = 0.47; % higher phi1 => yaut is further lower than y, so fewer default
              % and increases E(d/y)
rstar = 0.04 ;   
mu = 0.154 ;  
sigg = 2 ; 
alfa = 0.65 ;  %bargaining power of borrower, high alfa means higher default
betta = 0.85 ; 
coup = 0.06 ; % long-term bond, coupon rate
eta = 0.15 ;  % 1/eta quarters, long-term bond, average maturity
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
sdm = 0.0075 ;  %stdev of log prod shock 
mum = 1.01; %long run mean of log prod

[zgrid,pdfz] = tauchen(nz,muz,rhoz,sdz,width);
[mgrid,pdfm] = tauchen(nm,mum,rhom,sdm,width);

ns = nz*nm;
pdf_joint = kron(pdfm,pdfz); % Joint distribution, 
pdf_joint = pdf_joint ./ (((sum(pdf_joint,2)))*ones(1,ns)); 
z = zeros(ns,1) ;  m = zeros(ns,1) ; 
for im = 1:nm
    for iz = 1:nz 
        is = (im-1)*nz + iz ;
        z(is) = zgrid(iz) ;
        m(is) = mgrid(im) ; 
    end
end

%debt grid
bupper = 1.3;    blower = 0.2;
b = blower:(bupper-blower)/(nb-1):bupper;
b = b(:);

sigg_bp = 0.0005 ;  % taste shocks, for debt policy
sigg_defp = 0.0005 ; % for default
sigg_bpr = 0.0005 ;  % for renegotiation

para = [alfa, betta, phi0, phi1, phi2, sigg_bp, sigg_bpr, sigg_defp ] ;

%% --- matlab mex serial solve ---

if generate_mexserial

cfg = coder.config('mex');
cfg.GenerateReport = true;

vs_z = coder.typeof(z,[1000,1],1) ;
vs_m = coder.typeof(z,[1000,1],1) ;
vs_pdf_joint = coder.typeof(pdf_joint,[1000,1000],1) ;
vs_b = coder.typeof(b,[1000,1],1) ;

codegen -config cfg solve_benchmark -args {vs_z,vs_m,vs_b,vs_pdf_joint,para } -o solve_benchmarkVS_mex

end

if run_mexserial

[vp,vd,q_mex,bp,bpr,default,rr,totaltime,avgtime] = solve_benchmarkVS_mex( z,m,b,pdf_joint,para  ) ;
disp(['Mex serial time total: ', num2str(totaltime)]) ;
disp(['Mex serial time average: ', num2str(avgtime)]) ;

end
%% Mex parfor

if generate_mex

vs_z = coder.typeof(z,[1000,1],1) ;
vs_m = coder.typeof(z,[1000,1],1) ;
vs_pdf_joint = coder.typeof(pdf_joint,[1000,1000],1) ;
vs_b = coder.typeof(b,[1000,1],1) ;

cfg = coder.config('mex');
cfg.GenerateReport = true;
codegen -config cfg solve_benchmark_parfor -args {vs_z,vs_m,vs_b,vs_pdf_joint,para } -o solve_benchmarkVS_parfor_mex

cfg = coder.gpuConfig('mex');
cfg.GenerateReport = true;
codegen -config cfg solve_benchmark_cuda -args {vs_z,vs_m,vs_b,vs_pdf_joint,para } -o solver_benchmarkVS_cuda_mex

end

if run_mex

[vp,vd,q_mexparfor,bp,bpr,default,rr,totaltime,avgtime] = solve_benchmarkVS_parfor_mex( z,m,b,pdf_joint,para  ) ;

disp(['Mex parfor time total: ', num2str(totaltime)]) ;
disp(['Mex parfor time average: ', num2str(avgtime)]) ;

reset(gpuDevice)
[vp,vd,q_mexcuda,bp,bpr,default,rr,totaltime,avgtime] = solver_benchmarkVS_cuda_mex( z,m,b,pdf_joint,para) ;

disp(['Mex cuda time total: ', num2str(totaltime)]) ;
disp(['Mex cuda time average: ', num2str(avgtime)]) ;


diff = norm(q_mexparfor - q_mexcuda) 

end



