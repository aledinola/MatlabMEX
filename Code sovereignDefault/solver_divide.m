function [q,bp,vp,def,totaltime,avgtime] = solver_divide(b,z,m,pdf,para)

theta = para.theta ; 
betta = para.betta ;
sigg = para.sigg ;
rstar = para.rstar ;
phi0 = para.phi0 ;

ns = size(z,1);
nb = size(b,1);
[~,nb0] = min(abs(b));  % index of b=0 ?

    % Initialize
vp  = zeros(ns,nb);  % continue repaying
vd  = zeros(ns,1);   % default region
vo  = vd;
vp1 = vp;  
bp  = ones(ns,nb) ;  % debt policy function (indices)
q   = ones(ns,nb) / (1 + rstar) ; 
def = false(ns,nb) ;
def1 = false(ns,nb) ;

ua = ( (exp(z).*m *(1-phi0)).^(1-sigg) - 1) / (1-sigg)  ;

%%
tol = 1e-7;
diff = 1;
its  = 1;
    
timer = tic; % Start timer

while diff > tol && its < 1000

evp = m.^(1-sigg).*betta.*pdf*vp;

vd1 = ua + m.^(1-sigg).*betta.*pdf*(theta*vo + (1-theta)*vd);

w = b' .* q + exp(z).*m ;

for is = 1:ns
    [vp1(is,:), bp(is,:), def1(is,:)] = bmonoNonRec(b, w(is,:), evp(is,:), nb, vd1(is), def(is,:) ) ;
end

qnew = (1- pdf*def) / (1+rstar);

diff = max(abs(qnew(:) - q(:))) + max(abs(vp1(:) - vp(:))) + ...
           max(abs(vd1(:) - vd(:)));

vo  = vp1(:,nb0);
vp  = vp1;
vd  = vd1;
def = def1;
q   = qnew;

    if mod(its,50)==0
        fprintf('%5.0f ~ %8.10f \n', its, diff);
    end
    its = its+1;
end

totaltime = toc(timer);
avgtime   = totaltime/(its-1);
fprintf('# its%4.0f ~Time %8.8fs ~Avgtime %8.8fs \n', its-1, totaltime, avgtime);

end



function [Vrow, bprow, defrow] = bmonoNonRec(b, w, evpv, nb, vd_is, def_is)

    Vrow = zeros(1, nb);
    bprow = zeros(1, nb);
    defrow = zeros(1, nb);

    if def_is(1) == 1
        Vrow(1) = vd_is ;    bprow(1) = 1 ;
    else
        [Vrow(1), bprow(1)] = maxStep(1, nb, b(1), w, evpv);
    end

    if def_is(nb) == 1
        Vrow(nb) = vd_is ;    bprow(nb) = nb ;
    else
        [Vrow(nb), bprow(nb)] = maxStep(1, nb, b(nb), w, evpv);
    end

    stackSize = ceil(log2(nb-1)) + 2 ;
    L = zeros(stackSize,1); 
    U = zeros(stackSize,1);

    L(1) = 1;
    U(1) = nb;
    k    = 1;

    while true
        while U(k) ~= L(k)+1
            k = k + 1;
            L(k) = L(k-1);
            U(k) = floor((L(k-1) + U(k-1))/2);
            ib = U(k);  % m is the ib state index

            if def_is(ib) == 1
                Vrow(ib) = vd_is ;   bprow(ib) = ib ;
            else
                bp_ind_Low = bprow(L(k-1)) ;
                bp_ind_High = bprow(U(k-1)) ;
                [Vrow(ib), bprow(ib)] = maxStep(bp_ind_Low, bp_ind_High, b(ib), w, evpv);
            end
            
            if Vrow(ib) > vd_is
                defrow(ib) = 0 ;
            else
                defrow(ib) = 1 ;
            end
        end

        while true
            if k == 1; break; end
            if U(k) ~= U(k-1); break; end
            k = k - 1;
        end
        if k == 1 && U(k) == nb; break; end
        L(k) = U(k);
        U(k) = U(k-1);
    end
    
    if Vrow(1) > vd_is; defrow(1) = 0 ; else; defrow(1) = 1 ; end
    if Vrow(nb) > vd_is; defrow(nb) = 0 ; else; defrow(nb) = 1 ; end
end


function [bestVal, bestIp] = maxStep(ibp_min, ibp_max, bib, w, evpv)

    bestVal = -Inf;
    bestIp  = 1;

    for ibp = ibp_min:ibp_max

        % c1 = b(ibp) * qv(ibp) - b(ib) + yis ;
        c1 = w(ibp) - bib ;

        if c1 > 0
            val  = (c1^(1-2)-1) / (1-2) + evpv(ibp) ;
        else
            val = - Inf ;
        end

        if val > bestVal
            bestVal = val;
            bestIp  = ibp;
        end
    end

end



