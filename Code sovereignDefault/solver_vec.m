function [q,bp,vp,def,totaltime,avgtime] = solver_vec(b,z,m,pdf,para)

phi0 = para.phi0  ; 
rstar = para.rstar  ;  
theta = para.theta  ; 
sigg = para.sigg ; %intertemporal elasticity of consumption substitution 
betta = para.betta ;  %discount factor

ns = length(z) ;
nb = length(b) ;
n = ns*nb ;

[~,nb0] = min(abs(b));  b(nb0) = 0; %force the element closest to zero to be exactly zero

ua = ( (exp(z).*m *(1-phi0)).^(1-sigg) - 1) / (1-sigg)  ;

mtry = repmat(m,[1,nb]) ;
mtry = repmat(mtry,[nb,1]) ;
ztry = repmat(z,[1,nb]) ;
ztry = repmat(ztry,[nb,1]) ;

btry = repmat(b',[ns 1  nb]);
btry = reshape(btry,n,nb);

bptry = repmat(b',n,1);

%% Initialize the Value functions
vp = zeros(ns,nb);  %continue repaying
vd = zeros(ns,1); 

vo = vd;
vp1 = vp;  
vd1 = vd;
def = false(ns,nb);

bp = zeros(ns,nb); %debt policy function (expressed in indices)  
bp1 = bp;
q = ones(ns,nb)/(1+rstar); %q is price of debt; it is a function of  (y_t, d_{t+1}) 

%%
diff = 1;
tol = 1e-7;
its = 1;
maxits = 2000;
smctime   = tic;
totaltime = 0; avgtime = 0 ;

while diff > tol  &&  its < maxits

qtry = repmat(q,[nb 1]);

ctry =  bptry .* qtry - btry + exp(ztry).*mtry ;

utry = (ctry.^(1-sigg) -1)  / (1-sigg);

utry(ctry<=0) = -inf;

Evgood = m.^(1-sigg).*pdf * vp;
Evgood = repmat(Evgood,nb,1);

[vp1(:), bp1(:)] = max(utry + betta*Evgood, [], 2);

vd1 = ua + m.^(1-sigg).*betta.*pdf*(theta*vo + (1-theta)*vd);

def = vp1 < repmat(vd1,1,nb); 

qnew = (1- pdf*def)/(1+rstar);

vp_1 = max(repmat(vd1,1,nb),vp1);

diff = max(abs(qnew(:) - q(:))) + max(abs(vp_1(:) - vp(:)))...
    + max(abs(vd1(:) - vd(:)));

vo = vp_1(:,nb0);
vp = vp_1;
vd = vd1;
q = qnew;
bp = bp1;

totaltime = totaltime + toc(smctime);
avgtime   = totaltime/its;

  if mod(its, 50) == 0 || diff<=tol
      fprintf('%5.0f ~%8.8f ~%8.5fs ~%8.5fs \n', its, diff, totaltime, avgtime);
  end
    
  its = its+1;
  smctime = tic; % re-start clock

end %while dist>...



end

