function agg_vars = fun_agg_vars(a_grid, z_grid, StatDist, Params)

coder.gpu.kernelfun;

w       = Params.w;
r       = Params.r;
lambda  = Params.lambda;
delta   = Params.delta;
alpha   = Params.alpha;
upsilon = Params.upsilon;

n_a = length(a_grid);
n_z = length(z_grid);

% Preallocate intermediate results
profit = coder.nullcopy(zeros(n_a, n_z));
kstar  = coder.nullcopy(zeros(n_a, n_z));
lstar  = coder.nullcopy(zeros(n_a, n_z));

% Compute static entrepreneurial choices
coder.gpu.kernel()
for z_c = 1:n_z
    z = z_grid(z_c);
    for a_c = 1:n_a
        a = a_grid(a_c);
        [profit(a_c,z_c), kstar(a_c,z_c), lstar(a_c,z_c)] = solve_entre(a, z, w, r, lambda, delta, alpha, upsilon);
    end
end

% Allocate accumulation arrays
A_vec       = zeros(n_a, n_z);
E_vec       = zeros(n_a, n_z);
K_vec       = zeros(n_a, n_z);
L_vec       = zeros(n_a, n_z);
output_vec  = zeros(n_a, n_z);
extfin_vec  = zeros(n_a, n_z);

% Compute weighted aggregates
coder.gpu.kernel()
for z_c = 1:n_z
    z = z_grid(z_c);
    for a_c = 1:n_a
        a = a_grid(a_c);
        weight = StatDist(a_c, z_c);
        A_vec(a_c, z_c) = a * weight;

        if w <= profit(a_c, z_c)
            E_vec(a_c, z_c) = weight;
            K_vec(a_c, z_c) = kstar(a_c, z_c) * weight;
            L_vec(a_c, z_c) = lstar(a_c, z_c) * weight;
            output_vec(a_c, z_c) = z * ((kstar(a_c, z_c)^alpha) * (lstar(a_c, z_c)^(1 - alpha)))^(1 - upsilon) * weight;
            extfin_vec(a_c, z_c) = max(0, kstar(a_c, z_c) - a) * weight;
        end
    end
end

% Return aggregated outputs
agg_vars.A            = sum(A_vec(:));
agg_vars.K            = sum(K_vec(:));
agg_vars.L            = sum(L_vec(:));
agg_vars.entrepreneur = sum(E_vec(:));
agg_vars.output       = sum(output_vec(:));
agg_vars.extfin       = sum(extfin_vec(:));

end

