function [q,bp,vp,def,totaltime,avgtime] = solver_interp_cuda(b,z,m,pdf,para)

coder.gpu.kernelfun;

theta = para.theta ; 
betta = para.betta ;
sigg = para.sigg ;
rstar = para.rstar ;
phi0 = para.phi0 ;

% dpgrid = b';
ns = length(z) ;
nb = size(b,1);
[~,nb0] = min(abs(b)) ;

%Initialize the Value functions
vp = coder.nullcopy(zeros(ns,nb)) ;  %continue repaying
vd = coder.nullcopy(zeros(ns,1)) ; 
def = coder.nullcopy(false(ns,nb)) ; 
% vgood = vp ;
vo = vd;
vp1 = vp;  

bp = coder.nullcopy(zeros(ns,nb)) ; %debt policy function (expressed in indices) 
q = ones(ns,nb)/(1+rstar); %q is price of debt; it is a function of  (y_t, d_{t+1}) 
% u = zeros(1,nb) ;
bp_l = -0.05 ;
bp_u = b(nb) ;

ua = ( (exp(z).*m *(1-phi0)).^(1-sigg) - 1) / (1-sigg)  ;

%%
diff = 1;
tol = 1e-7;
its = 1;

timer = tic;  % <----- Start the timer

while diff > tol && its < 1000

evp = m.^(1-sigg).*betta.*pdf*vp;

vd1 = ua + m.^(1-sigg).*betta.*pdf*(theta*vo + (1-theta)*vd);

w = b' .* q + exp(z).*m ;

coder.gpu.kernel()
for is = 1:ns
    for ib = 1:nb
        if def(is,ib) == 0
            myfun = @(x) rhs_bellman(x, w(is,:) - b(ib), b, evp(is,:)) ;
            [xf, fval] = brent_min(myfun,bp_l,bp_u,1e-7,400,400);
            vp1(is,ib) = - fval ;
            bp(is,ib) = xf ;
        end
    end
end

coder.gpu.kernel()
for is = 1:ns
    for ib = 1:nb
        if def(is,ib) == 1
            vp1(is,ib) = vd1(is) ;
            bp(is,ib) = 0 ;
        else
            if vp1(is,ib) > vd1(is)
                def(is,ib) = 0 ;
            else 
                def(is,ib) = 1 ;
            end
        end
    end
end

% def = vp1 <= vd1 ; 

qnew = (1- pdf*def) / (1+rstar);

diff = max(abs(qnew(:) - q(:))) + max(abs(vp1(:) - vp(:))) ...
    + max(abs(vd1(:) - vd(:))) ;

vo = vp1(:,nb0);
vp = vp1;
vd = vd1;
q = qnew;

if mod(its, 30) == 0 
  fprintf('%5.0f ~ %8.10f \n', its, diff);
end
its = its + 1;
end

totaltime = toc(timer);
avgtime   = totaltime/(its-1);
fprintf('# its%4.0f ~Time %8.8fs ~Avgtime %8.8fs \n', its-1, totaltime, avgtime);
end


%% SUBFUNCTIONS 
function F = rhs_bellman(bprime,c_vec,b_grid,evp_y)

coder.gpu.kernelfun;

coder.gpu.constantMemory(b_grid); 

eta = 2 ;

c1 = interp1_scal(b_grid, c_vec, bprime);
if c1 <= 0
    F = Inf;
else
    evp_y_interp = interp1_scal(b_grid,evp_y,bprime) ;
    F = - (c1^(1-eta)-1)/(1-eta) - evp_y_interp ;
end

end 
