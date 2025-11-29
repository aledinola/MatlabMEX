function [V,Policy,Params] = fun_VFI_vec(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions)

coder.gpu.kernelfun;

Params.r = p_eqm(1);
Params.w = p_eqm(2);

verbose   = vfoptions.verbose ;

%% 1 First solve the value function

n_a = length(a_grid) ;
n_z = length(z_grid) ;

if verbose>=1
    disp('Start Value Function Iteration')
end

r       = p_eqm(1);
w       = p_eqm(2);
lambda  = Params.lambda;
delta   = Params.delta;
alpha   = Params.alpha;
upsilon = Params.upsilon;
crra    = Params.crra;
beta    = Params.beta;

%% 1.1 the return matrix

%tic

profit = solve_entre_vec(a_grid,z_grid.',w,r,lambda,delta,alpha,upsilon);
% cash is a × z
cash = max(w,profit) + (1+r)*a_grid; % cash depends only on (a,z)

cash = reshape(cash,1,n_a,n_z);  % 1  × a × z
cons = cash - a_grid;            % a' × a × z
ReturnMatrix = - Inf(size(cons));
mask = cons>0;
ReturnMatrix(mask) = cons(mask).^(1-crra)/(1-crra);
    
%time_ret = toc;

%% 1.1 the value and policy function

V0     = coder.nullcopy(zeros(n_a,n_z)) ; % Initial guess V0
V      = coder.nullcopy(zeros(n_a,n_z)) ;
Policy = coder.nullcopy(zeros(n_a,n_z)) ;

err  = vfoptions.tolerance+1;
iter = 1;

pi_z_tranposed = pi_z.';
% Build subscript grids for (a_c, z_c)
iAvec = repmat((1:n_a).',n_z,1);    % n_a*n_z × 1
iZvec = repelem((1:n_z).', n_a, 1); % n_a*n_z × 1


%tic
while err > vfoptions.tolerance

    EV = V0*beta*pi_z_tranposed; %EV(a',z)

    entireRHS = ReturnMatrix + reshape(EV,n_a,1,n_z);
    [max_val,max_ind] = max(entireRHS,[],1);
    %F_temp(a_c,z_c) = ReturnMatrix(max_ind,a_c,z_c);
    Policy = reshape(max_ind,n_a*n_z,1);
    V      = reshape(max_val,n_a,n_z);

    % -------------------------- Howard ----------------------------------%
    % Linear indices of the elements you want in ReturnMatrix
    linIdx = Policy + (iAvec-1)*n_a + (iZvec-1)*n_a*n_a;
    % Pull them out and reshape to n_a × n_z
    F_temp = reshape(ReturnMatrix(linIdx), n_a, n_z); 

    %Policy = reshape(Policy,n_a*n_z,1);

    for h_c=1:vfoptions.howards
        EVh = V*beta*pi_z_tranposed;
        lin = Policy+(iZvec-1)*n_a;
        V = F_temp+reshape(EVh(lin),n_a, n_z);   
    end %end howards
    % --------------------------------- ----------------------------------%
    
    % Update
    err = max(abs(V(:)-V0(:)));
    % if verbose == 2
    %     fprintf('iter = %4.0f, err = %f \n',iter,err)
    % end
    V0 = V;
    iter = iter+1;

end %end while

Policy = reshape(Policy,n_a,n_z);

%time_vfi = toc;

% if verbose >= 1
%     fprintf('Time return matrix:       %8.6f \n',time_ret);
%     fprintf('Time vfi:                 %8.6f \n',time_vfi);
%     fprintf('Time return matrix + vfi: %8.6f \n',time_ret+time_vfi);
% end

end %end function

