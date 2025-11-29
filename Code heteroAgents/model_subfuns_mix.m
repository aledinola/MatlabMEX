function [val,V,Policy,StatDist] = model_subfuns_mix(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions, simoptions, heteroagentoptions)

% the VFI should be compiled into CUDA
% [V,Policy,Params] = fun_VFI_cuda_mex(p_eqm, a_grid, z_grid, pi_z,Params,vfoptions) ;
[ReturnMatrix,Params,time_ret] = fun_return_cuda_mex(p_eqm, a_grid, z_grid, Params, vfoptions) ;

[V, Policy] = fun_VFIonly_parfor_mex(a_grid, z_grid, pi_z, Params, vfoptions, ReturnMatrix, time_ret) ;

% this function compiled into Mex parfor
% Policy is (n_a,n_z)
[StatDist, GE_eqns_vec] = fun_dist_agg(Policy, a_grid, z_grid, pi_z,Params, simoptions) ;

val = sum(GE_eqns_vec.^2); % the val to be minimized

%%
% Display prices and residuals (only on CPU)
if heteroagentoptions.verbose==1

    fprintf('Current GE prices: \n') ; 
    fprintf('    GE prices: r = %8.6f \n', p_eqm(1) );
    fprintf('    GE prices: w = %8.6f \n', p_eqm(2) );

    fprintf('Current GeneralEqmEqns: \n') ; 
    fprintf('    GE capital: %8.6f \n', GE_eqns_vec(1) );
    fprintf('    GE labor: %8.6f \n', GE_eqns_vec(2) );
    fprintf('-----------------------------------------\n');
end


end

