function [v, pol_kp] = rbc_solver_interp_gpu(c0, k, pdfz, tol)
%#codegen
% Solve the stochastic growth model with VFI and linear interpolation,
% using Brent minimization for the choice of next-period capital k'.
%
% Entry point for GPU MEX (GPU Coder).

coder.gpu.kernelfun;

%% Parameters
maxit = 2000;   % Maximum number of iterations

beta  = 0.95;   % Discount factor
eta   = 2.0;    % Risk aversion parameter

%% Dimensions and grids
[nk, nz] = size(c0);
k_min = k(1);
k_max = k(nk);

%% Preallocate value and policy functions
v      = coder.nullcopy(zeros(nk, nz, 'like', c0)); % value function
v0     = coder.nullcopy(zeros(nk, nz, 'like', c0)); % previous iterate
pol_kp = coder.nullcopy(zeros(nk, nz, 'like', c0)); % policy for k'

%% Value function iteration
diff = 1.0;
its  = int32(1);

while (diff > tol) && (its < maxit)

    % Expected discounted continuation value:
    % ev(k,z) = beta * sum_{z'} v(k,z') * pdfz(z,z')
    % Note: pdfz' has size (nz x nz), so v * pdfz' is (nk x nz)
    ev = beta * (v * pdfz.');

    % Flattened loop over all (k,z) pairs: idx = 1, ..., nk*nz
    coder.gpu.iterations(nk * nz);   % heuristic number of iterations
    coder.gpu.kernel();
    for idx = 1:(nk * nz)

        % Recover indices for exogenous (iz) and endogenous (ik) states
        iz = floor((idx - 1) / nk) + 1;          % z index
        ik = idx - (iz - 1) * nk;               % k index

        EV_z   = ev(:, iz);                     % continuation value over k' for this z
        wealth = c0(ik, iz);                    % total resources today

        % Impose k' in [k_min, min(k_max, wealth)]
        kp_lb = k_min;
        kp_ub = min(k_max, wealth) - 1e-8;

        % If wealth is extremely small, enforce a strictly positive interval
        if kp_ub <= kp_lb
            kp_ub = kp_lb + 1e-8;
        end

        % Brent minimization over k'
        myfun       = @(x) rhs_bellman(x, wealth, k, EV_z, eta);
        [xf, fval]  = brent_min(myfun, kp_lb, kp_ub, 1e-8, 500, 500);

        % Remember: rhs_bellman returns the negative of the value
        v0(ik, iz)     = -fval;
        pol_kp(ik, iz) = xf;

    end % for idx


    % Check convergence (max norm)
    diff = max(abs(v(:) - v0(:)));

    % Progress print only in normal MATLAB (not in generated MEX)
    if coder.target('MATLAB')
        if mod(its, 60) == 0
            fprintf('its = %d, diff = %e\n', its, diff);
        end
    end

    % Update value function
    v   = v0;
    its = its + 1;

end % while

end % function rbc_solver_interp_gpu

% -------------------------------------------------------------------------
function F = rhs_bellman(kprime, wealth, k_grid, EV_z, eta)
% RHS of the Bellman equation for given k' (used inside Brent).
% This calls user-written function interp1_scal for interpolation.
%
% Returns negative of the objective, since brent_min performs minimization.

coder.inline('always');

% Consumption implied by k'
c1 = wealth - kprime;

% Interpolate EV_z at k'
EV_z_interp = interp1_scal(k_grid, EV_z, kprime);

% CRRA utility + continuation value
u = c1^(1 - eta) / (1 - eta);
F = -(u + EV_z_interp);   % negative because brent_min minimizes

end % function rhs_bellman
