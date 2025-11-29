function [q,bp,vp,def,totaltime,avgtime] = solver_bf_parfor(b,z,m,pdf,para)

theta = para.theta ; 
betta = para.betta ;
sigg = para.sigg ;
rstar = para.rstar ;
phi0 = para.phi0 ;

ns = size(z,1);
nb = size(b,1);
[~,nb0] = min(abs(b));

%Initialize the Value functions
vp = zeros(ns,nb);  %continue repaying
vd = zeros(ns,1); 
def = false(ns,nb); 
vo = vd;
vp1 = vp;  

bp = zeros(ns,nb); %debt policy function (expressed in indices)  
q = ones(ns,nb)/(1+rstar); %q is price of debt; it is a function of  (y_t, d_{t+1}) 

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

parfor is = 1:ns
    for ib = 1:nb
        if def(is,ib) == 0
            c1 = w(is,:) - b(ib) ;
            u = (c1.^(1-sigg) -1) / (1-sigg) ;
            u(c1<=0) = - Inf ;
            [vp1(is,ib), bp(is,ib)] = max( u + evp(is,:)) ;
        end
    end
end

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

qnew = (1- pdf*def) / (1+rstar);

diff = max(abs(qnew(:) - q(:))) + max(abs(vp1(:) - vp(:)))...
    + max(abs(vd1(:) - vd(:)));

vo = vp1(:,nb0);
vp = vp1;
vd = vd1;
q = qnew;

if mod(its, 50) == 0 
  fprintf('%5.0f ~ %8.10f \n', its, diff);
end

its = its + 1;

end

totaltime = toc(timer);
avgtime   = totaltime/(its-1);

fprintf('# its%4.0f ~Time %8.8fs ~Avgtime %8.8fs \n', its-1, totaltime, avgtime);

end

