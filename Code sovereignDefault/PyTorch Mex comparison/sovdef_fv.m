clear

format compact
reset(gpuDevice)

%%
nb = 500;   % # of grid points for debt 

ygrid = load('logy_grid.txt');
Py = load('P.txt');

ny = length(ygrid) ;
ygrid = exp(ygrid);

para.betta = 0.953 ;
para.sigg = 2 ;
para.rstar = 0.017 ;
para.theta = 0.282;

Bgrid = linspace(-.45, 0, nb);

ymean = mean(ygrid);
def_y = min(0.969 * ymean, ygrid);

%% 

[V,Vc,Vd,Q,default_prob] = fv_solver(Bgrid,ygrid,Py,def_y,para);

%%
% 
% cfg = coder.gpuConfig('mex');
% cfg.GenerateReport = true;
% codegen -config cfg fv_solver -args {Bgrid,ygrid,Py,def_y,para} -o fv_solver_cuda
% 
[V,Vc,Vd,Q,default_prob] = fv_solver_cuda(Bgrid,ygrid,Py,def_y,para);


