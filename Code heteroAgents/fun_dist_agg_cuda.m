function [StatDist, GE_eqns_vec] = fun_dist_agg_cuda(Policy, a_grid, z_grid, pi_z, Params, simoptions)

w       = Params.w;
r       = Params.r;
lambda  = Params.lambda;
delta   = Params.delta;
alpha   = Params.alpha;
upsilon = Params.upsilon;

n_a = length(a_grid) ;
n_z = length(z_grid) ;

%% 2. Now create the distribution

verbose   = simoptions.verbose;

% Initial condition for distribution iteration
mu0 = ones(n_a,n_z)/(n_a*n_z);

if verbose>=1
    disp('Start Distribution')
end

% Preallocate output
StatDist = zeros(n_a,n_z) ; 
err  = simoptions.tolerance + 1;
iter = int32(1) ;
tic
while (err > simoptions.tolerance && iter <= simoptions.maxit)

    StatDist(:,:) = 0 ;
    
    for z_c=1:n_z
        for a_c=1:n_a
            aprime_opt = Policy(a_c,z_c) ;
            StatDist(aprime_opt,z_c) = StatDist(aprime_opt,z_c) + mu0(a_c,z_c) ;
        end
    end

    % mu(a',z)*pi_z(z,z')---> mu(a',z')
    StatDist = StatDist*pi_z;

    err = max(abs(StatDist(:)-mu0(:)));
    
    % if simoptions.verbose == 2
    %     fprintf('iter = %d, err = %f \n',iter,err)
    % end
    
    % Update
    mu0  = StatDist;
    iter = iter+1;
end %end while
time_dist = toc ;

if err > simoptions.tolerance
    warning('Distribution did not converge')
end

check_sum = sum(StatDist(:));
if abs(check_sum-1.0) > 1e-7
    fprintf('check_sum = %8.6f \n',check_sum)
    error('Distribution does not sum to one')
end
StatDist = StatDist/check_sum;

if verbose>=1
    fprintf('time_dist = %8.6f \n',time_dist);
end

%% 3. compute the aggregate

% Preallocate intermediate results
% profit = coder.nullcopy(zeros(n_a, n_z));
% kstar  = coder.nullcopy(zeros(n_a, n_z));
% lstar  = coder.nullcopy(zeros(n_a, n_z));

% Allocate accumulation arrays
A_vec       = zeros(n_a, n_z);
E_vec       = zeros(n_a, n_z);
K_vec       = zeros(n_a, n_z);
L_vec       = zeros(n_a, n_z);

% Compute static entrepreneurial choices
for z_c = 1:n_z
    z = z_grid(z_c);
    for a_c = 1:n_a
        a = a_grid(a_c);

        [profit, kstar, lstar] = solve_entre(a, z, w, r, lambda, delta, alpha, upsilon);
        weight = StatDist(a_c, z_c);
        A_vec(a_c, z_c) = a * weight;

        if w <= profit
            E_vec(a_c, z_c) = weight;
            K_vec(a_c, z_c) = kstar * weight;
            L_vec(a_c, z_c) = lstar * weight;
        end
    end
end


% Return aggregated outputs
agg_vars_A            = sum(A_vec(:));
agg_vars_K            = sum(K_vec(:));
agg_vars_L            = sum(L_vec(:));
agg_vars_entrepreneur = sum(E_vec(:));

% GE(1): capital demand minus capital supply
GE_K = agg_vars_K - agg_vars_A;

% GE(2): labor demand minus labor supply, suppy is just fraction of workers (who each exogneously supply endowment 1 of labor)
GE_L = agg_vars_L - (1 - agg_vars_entrepreneur);

GE_eqns_vec = [GE_K, GE_L];  

end

