function [vp,vd,q,bp,bpr,default,rr,totaltime,avgtime] = solve_benchmark_parfor(z,m,b,pdf,para) 


% para = [alfa, betta, phi1, phi2, sigg_bp, sigg_bpr, sigg_defp ] ;

alfa = para(1) ;  betta = para(2) ;  phi0 = para(3) ; phi1 = para(4) ;  
phi2 = para(5) ;  sigg_bp = para(6) ;  sigg_bpr = para(7) ; sigg_defp = para(8) ;

rstar = 0.04;   
mu= 0.154;  
coup = 0.058; %0.03, long-term bond, coupon rate
eta = 0.15; %long-term bond, average maturity
sigg = 2 ;

ns = length(z) ;
nb = length(b);

vp = zeros(ns,nb);  
vd = zeros(ns,1); 
V = zeros(ns,nb); 
vpnew = zeros(ns,nb); 

bp = zeros(ns,nb); %debt policy function (expressed in indices)  
bpr = zeros(ns,1); % debt policy (index) when decided renegotiate (right after every default)
q = ones(ns,nb)/(1+rstar); %q is price of debt; it is a function of  (y_t, d_{t+1}) 
qnew = zeros(ns,nb);
rr = 0.5*ones(ns,nb)/(1+rstar);
probdef = zeros(ns,nb); 

WW = zeros(nb,ns) ;
Gamma = zeros(ns,1) ;
Dcre = zeros(ns,nb) ;

% pdf = sparse(pdf_joint) ;

ua = ( (exp(z).*m *(1-phi0)).^(1-sigg) - 1) / (1-sigg)  ;

% to incorporate taste shocks
epsi = 10e-16;
cv_bp = sigg_bp*log(epsi); % critical value
cv_bpr = sigg_bpr*log(epsi);  % critical value
probDcre = zeros(ns,1);
probVp = zeros(ns,1);

smctime   = tic;
totaltime = 0;
avgtime = 0;

dist = 1;    vaut = zeros(ns,1);  
while dist > 1e-8
    vautnew = ua + m.^(1-sigg).*pdf*betta*vaut ; 
    dist = max(abs(vautnew(:)-vaut(:))) ;
    vaut = vautnew ;
end  

%%  the main VFI
diff = 1;  tol = 1e-7;
its = 1;   maxits = 2000;

while diff > tol && its< maxits

    evp = m.^(1-sigg).*betta.*pdf*V; 

parfor is = 1:ns  % change the loop to y first d second

    W = zeros(1,nb,'double') ;

    for ib = 1:nb

        for i = 1: nb

            if q(is,i) >= 0.45

            c = exp(z(is))*m(is) - (eta+(1-eta)*coup)*b(ib) + q(is,i)*(b(i)*m(is) - (1-eta)*b(ib)) ;
              
              if c <= 0
                  W(i) = - Inf; 
              else
                  W(i) = 1 - c.^(-1) + evp(is,i) ;
              end

            else
                W(i) = - Inf;
            end

        end

        [vpnew(is,ib), bp(is,ib)]  = max( W ) ;

        sumExp = 0;
        sumExpQ = 0;
        for i = 1:nb
            temp = W(i) - vpnew(is, ib) - cv_bp;
            if temp > 0
                theExp = exp((temp + cv_bp) / sigg_bp);  % Compute theExp
                sumExp = sumExp + theExp;               % Accumulate theExp
                sumExpQ = sumExpQ + theExp * q(is, i);  % Accumulate theExpQ
            end
        end
        qnew(is, ib) = eta + (1 - eta) * (coup + sumExpQ / sumExp);
    end
end


parfor is = 1:ns 
    maxWW = -inf;  % To store the maximum value of WW

    for ib = 1:nb 
        Dsov = max(0, vpnew(is,ib) - vaut(is));
        Dcre(is,ib) = (eta + (1-eta)*(coup + q(is,bp(is,ib)) ) ).*b(ib);
        WW(ib,is) = Dsov^alfa * Dcre(is,ib)^(1-alfa);

        if WW(ib,is) > maxWW
            maxWW = WW(ib,is);  % Update maximum value
        end
    end
    Gamma(is) = maxWW; 
end


parfor is = 1:ns

    sumExp = 0;
    probDcre_is = 0;
    probVp_is = 0;

    for ib = 1:nb
        temp = WW(ib, is) - Gamma(is) - cv_bpr;
        if temp > 0
            theExp = exp((temp + cv_bpr) / sigg_bpr);
            sumExp = sumExp + theExp;  % Accumulate sums directly
            probDcre_is = probDcre_is + theExp * Dcre(is, ib);
            probVp_is = probVp_is + theExp * vpnew(is, ib);
        end
    end

    if sumExp > 0
        probDcre(is) = probDcre_is / sumExp;
        probVp(is) = probVp_is / sumExp;
    else
        probDcre(is) = 0;
        probVp(is) = 0;
    end
end

vdnew = ua + m.^(1-sigg).*betta.*pdf*(mu*probVp + (1-mu)*vd);

probdef = 1./(1 + exp((vpnew - vdnew)/sigg_defp)) ;

rrnew = pdf*((1-mu)*rr + mu*probDcre./b')/(1 + rstar); 

qnew = pdf*(probdef.*rrnew + (1-probdef).*qnew)/(1 + rstar);

diff = max(max(abs(qnew-q))) + max(abs(vpnew(:)-vp(:))) + max(max(abs(vdnew-vd)));

vp = vpnew ; 
vd = vdnew ;
V = max(vp,repmat(vd,1,nb));
q = qnew  ;
rr = rrnew ;

totaltime = totaltime + toc(smctime);
avgtime   = totaltime/its;
if mod(its, 30) == 0 || diff<=tol; fprintf('%8.0f ~%8.8f ~%8.5fs ~%8.5fs \n', its, diff, totaltime, avgtime); end
its = its+1;
smctime = tic; % re-start clock

end % end while
%%

default = vp < repmat(vd,1,nb);

%%
parfor is = 1:ns 

    maxWW = -inf;  % Initialize maximum WW
    maxIdx = 1;    % Initialize index for maximum WW

    for ib = 1:nb 

        Dsov = max(0, vpnew(is, ib) - vaut(is));
        Dcre1 = (eta + (1 - eta) * (coup + q(is, bp(is, ib)))) * b(ib);

        WW1 = Dsov^alfa * Dcre1^(1 - alfa);

        if WW1 > maxWW
            maxWW = WW1;
            maxIdx = ib;
        end
    end
    bpr(is) = maxIdx ;

end



end % end function


