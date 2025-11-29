function [V,Policy,Params] = fun_VFI_parfor(p_eqm, a_grid, z_grid, pi_z, Params, vfoptions)

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
ReturnMatrix = zeros(n_a,n_a,n_z) ;
%C_a_grid = parallel.pool.Constant(a_grid);
tic

parfor z_c = 1:n_z
    z = z_grid(z_c);
    for a_c=1:n_a
        a = a_grid(a_c);
        profit = solve_entre(a,z,w,r,lambda,delta,alpha,upsilon);
        cash = max(w,profit) + (1+r)*a; % cash depends only on (a,z)
        cons = cash - a_grid; % Now introduce a'
        F = - Inf(size(cons));
        F(cons>0) = cons(cons>0).^(1-crra)/(1-crra);
        ReturnMatrix(:,a_c,z_c) = F;
        % for aprime_c = 1 : n_a  % Now introduce a'
        %    cons = cash(a_c,z_c) - a_grid(aprime_c);   
        %    if cons > 0
        %        ReturnMatrix(aprime_c,a_c,z_c) = (cons^(1-crra))/(1-crra);
        %    else
        %        ReturnMatrix(aprime_c,a_c,z_c) = - Inf ;
        %    end %end if
        % end %end aprime
    end %end a
end %end z

time_ret = toc;

%% 1.1 the value and policy function

V0 = zeros(n_a,n_z) ; % Initial guess V0
V      = zeros(n_a,n_z) ;
Policy = zeros(n_a,n_z) ;
F_temp = zeros(n_a,n_z);

err  = vfoptions.tolerance+1;
iter = 1;

pi_z_transposed = pi_z';

tic
while err > vfoptions.tolerance

    EV = V0*beta*pi_z_transposed; %EV(a',z)

    parfor z_c=1:n_z
        for a_c=1:n_a
            entireRHS = ReturnMatrix(:,a_c,z_c) + EV(:,z_c);
            [max_val,max_ind] = max(entireRHS);
            F_temp(a_c,z_c) = ReturnMatrix(max_ind,a_c,z_c);
            Policy(a_c,z_c) = max_ind;
            V(a_c,z_c)      = max_val;
        end
    end

    % -------------------------- Howard ----------------------------------%
    h_c = 1 ;
    while h_c < vfoptions.howards
        EVh = V*beta*pi_z_transposed;
        for z_c=1:n_z
            for a_c=1:n_a
                aprime_opt = Policy(a_c,z_c);
                V(a_c,z_c) = F_temp(a_c,z_c) + EVh(aprime_opt,z_c);
            end
        end
        h_c = h_c + 1 ;
    end %end howards
    % --------------------------------- ----------------------------------%
    
    % Update
    err = max(abs(V(:)-V0(:)));
    V0 = V;
    iter = iter+1;

end %end while
time_vfi = toc;

if verbose >= 1
    fprintf('Time return matrix:       %8.6f \n',time_ret);
    fprintf('Time vfi:                 %8.6f \n',time_vfi);
    fprintf('Time return matrix + vfi: %8.6f \n',time_ret+time_vfi);
end


end %end function

