function [v,pol_kp]  = rbc_solver_interp(c0,k,pdfz,tol)

maxit = 2000;
diff  = 1;
its   = int32(1);

beta = 0.95;      % Discount rate
eta  = 2;

[nk,nz] = size(c0);
k_min = k(1);
k_max = k(nk);

v = zeros(nk,nz);     % Value Function
v0 = zeros(nk,nz);    % v at the previous iteration
pol_kp = zeros(nk,nz);

while diff > tol && its < maxit

    % In Fortran it would be:
    % EV(k',z) = matmul(v(k',z'),transpose(pdf(z,z')))
    ev = beta*v*pdfz';

    parfor iz = 1:nz % z today (exogenous state)
        EV_z = ev(:,iz);
        for ik = 1:nk % k today (endo state)

            wealth = c0(ik,iz);
            % Impose k' in [k_min, min(k_max, wealth)]
            kp_lb = k_min;
            kp_ub = min(k_max, wealth) - 1e-8;

            % If wealth is extremely small, enforce a strictly positive interval
            if kp_ub <= kp_lb
                kp_ub = kp_lb + 1e-8;
            end

            myfun = @(x) rhs_bellman(x,wealth,k,EV_z,eta);
            [xf,fval] = brent_min(myfun,kp_lb,kp_ub,1e-8,500,500);
            v0(ik,iz) = -fval;
            pol_kp(ik,iz) = xf;

        end %end ik
    end %end iz

    diff = max(max(abs(v-v0)));   % Check convergence:

    %if mod(its, 60) == 0
    %    fprintf('%5.0f ~ %8.10f \n', its, diff);
    %end
    if mod(its, 60) == 0
        fprintf('its = %d, diff = %f \n',its,diff)
    end

    % Update
    v   = v0;
    its = its + 1;

end %end while

end %end function

% %--------------------------- SUBFUNCTIONS --------------------------------%
function F = rhs_bellman(kprime,wealth,k_grid,EV_z,eta)
% This calls user-written function interp1_scal

c1 = wealth-kprime;
EV_z_interp = interp1_scal(k_grid,EV_z,kprime);
F = c1^(1-eta)/(1-eta) + EV_z_interp;
F = -F;

end %end subfunction rhs_bellman

