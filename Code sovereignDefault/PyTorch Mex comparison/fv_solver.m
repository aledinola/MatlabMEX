function [V,Vc,Vd,Q,default_prob] = fv_solver(Bgrid,ygrid,Py,def_y,para)

coder.gpu.kernelfun;

theta = para.theta ; 
betta = para.betta ;
sigg = para.sigg ;
rstar = para.rstar ;

ny = length(ygrid) ;
nb = length(Bgrid) ;

Vd = coder.nullcopy(zeros(ny, 1));
Vc = coder.nullcopy(zeros(ny, nb)) ;
V = coder.nullcopy(zeros(ny, nb)) ;
Q = ones(ny, nb) * .95;

y = reshape(ygrid, ny, 1, 1);
B = reshape(Bgrid, 1, nb, 1);
Bnext = reshape(Bgrid, 1, 1, nb);

zero_ind = nb;

ua = def_y.^(1 - sigg) / (1 - sigg);

% u = @(c) c.^(1 - sigg) / (1 - sigg);

repeats = 1000 ;
V_ = V ;
 
timer  = tic;
tol = 1e-7;

for iteration = 1:repeats

        EV = Py * V;
        EVd = Py * Vd;
        EVc = Py * Vc;

        Vd_target = ua + betta * (theta * EVc(:, zero_ind) + (1 - theta) * EVd);
        Vd_target = reshape(Vd_target, ny, 1);

        Qnext = reshape(Q, ny, 1, nb);

        c = max(y - Qnext .* Bnext + B, 1e-14);
        EV = reshape(EV, ny, 1, nb);
        m =  c.^(1 - sigg) / (1 - sigg) + betta * EV;
        Vc_target = max(m, [], 3);

        default_states = Vd > Vc;
        default_prob = Py * default_states;
        Q_target = (1 - default_prob) / (1 + rstar);

        V_target = max(Vc_target, Vd_target);

        % diff = max(abs(V_target(:) - V(:))) ;

        V = V_target;
        Vc = Vc_target;
        Vd = Vd_target;
        Q = Q_target;

        if mod(iteration, 50) == 0 && iteration > 10

            V_new_ = V ;
            diff = max(abs(V_(:) - V_new_(:))) ;
            V_ = V_new_ ;

            fprintf('%5.0f ~ %8.10f \n', iteration, diff);

            if diff  < tol
               totaltime = toc(timer);
               avgtime   = totaltime/iteration;
               fprintf('FV # its%4.0f ~Time %8.8fs ~Avgtime %8.8fs \n', iteration, totaltime, avgtime);
               break
            end
        end
end



end

