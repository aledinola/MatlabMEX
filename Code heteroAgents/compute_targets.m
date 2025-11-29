function [Outputs] = compute_targets(a_grid, z_grid, pi_z, Policy, StatDist, Params, p_eqm)

coder.gpu.kernelfun;

Params.r = p_eqm(1);
Params.w = p_eqm(2);

n_a = length(a_grid) ;
n_z = length(z_grid) ;

%% Compute some aggregate variables
agg_vars = fun_agg_vars(a_grid, z_grid, StatDist, Params);

%% GE_eqns_vec = f_GeneralEqmEqns(agg_vars);

GE_eqns_vec = zeros(1,2) ;

% GE(1): capital demand minus capital supply
GE_eqns_vec(1) = agg_vars.K - agg_vars.A; 

% GE(2): labor demand minus labor supply, suppy is just fraction of workers (who each exogneously supply endowment 1 of labor)
GE_eqns_vec(2) = agg_vars.L - (1-agg_vars.entrepreneur); 


%% Compute some policy functions [pol_e, pol_labor, pol_earnings] = fun_ValuesOnGrid(a_grid, z_grid, Params) ;

w       = Params.w;
r       = Params.r;
lambda  = Params.lambda;
delta   = Params.delta;
alpha   = Params.alpha;
upsilon = Params.upsilon;

% Compute some policy functions
pol_e        = coder.nullcopy(zeros(n_a,n_z)) ;
pol_labor    = coder.nullcopy(zeros(n_a,n_z)) ;
pol_earnings = coder.nullcopy(zeros(n_a,n_z)) ;

coder.gpu.kernel()
for z_c=1:n_z
    z = z_grid(z_c);
    for a_c=1:n_a
        a = a_grid(a_c);
        pol_e(a_c,z_c) = f_entrepreneur(a,z,w,r,lambda,delta,alpha,upsilon);
        pol_labor(a_c,z_c) = f_labordemand(a,z,w,r,lambda,delta,alpha,upsilon);
        pol_earnings(a_c,z_c) = f_earnings(a,z,w,r,lambda,delta,alpha,upsilon);
    end
end


%% Compute exit rate
[exit_E_to_W, entry_W_to_E, ~] = fun_entry_exit(pi_z, StatDist, Policy, pol_e, n_a, n_z) ;


%% Compute top 10% employment (i.e. share of employment in the largest 10% of establishments)
% [top10_empl] = fun_entre_top(pol_labor, pol_e, StatDist, n_a, n_z) ;
N   = n_a * n_z;

lab  = reshape(pol_labor, [N, 1]);
entre = reshape(pol_e, [N, 1]);
mass  = reshape(StatDist, [N, 1]);

lab(entre == 0)  = 0;
mass(entre == 0) = 0;

top10_empl = percentile_share(lab, mass, 0.90);


%% Compute top 5% earnings
% [top5_earnings] = fun_earnings_top(pol_earnings, StatDist, n_a, n_z);

value = reshape(pol_earnings, [N,1]);
mass  = reshape(StatDist, [N,1]);

top5_earnings = percentile_share(value, mass, 0.95);

%% Aggregate quantities and prices
Outputs.share_entre = agg_vars.entrepreneur;
Outputs.extfin      = agg_vars.extfin;
Outputs.Y           = agg_vars.output;
Outputs.r           = Params.r;
Outputs.w           = Params.w;
Outputs.GE_eqns_vec = GE_eqns_vec;
Outputs.K           = agg_vars.K;
Outputs.L           = agg_vars.L;
Outputs.K_Y         = agg_vars.K/agg_vars.output;
Outputs.extfin_Y    = agg_vars.extfin/agg_vars.output;
Outputs.exit_E_to_W   = exit_E_to_W;
Outputs.entry_W_to_E  = entry_W_to_E;
Outputs.top10_empl    = top10_empl;
Outputs.top5_earnings = top5_earnings;

end %end function
