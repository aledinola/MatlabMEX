function [V, Policy] = fun_VFIonly_parfor(a_grid, z_grid, pi_z, Params, vfoptions, ReturnMatrix, time_ret)

% Params.r = p_eqm(1);
% Params.w = p_eqm(2);

verbose   = vfoptions.verbose ;
%% 1 First solve the value function

n_a = length(a_grid) ;
n_z = length(z_grid) ;

% if verbose>=1
%     disp('Start Value Function Iteration')
% end

% r       = p_eqm(1);
% w       = p_eqm(2);
% lambda  = Params.lambda;
% delta   = Params.delta;
% alpha   = Params.alpha;
% upsilon = Params.upsilon;
% crra    = Params.crra;
beta    = Params.beta;

% %% 1.1 the return matrix
% ReturnMatrix = coder.nullcopy(zeros(n_a,n_a,n_z)) ;
% cash = coder.nullcopy(zeros(n_a,n_z)) ;
% 
% tic
% coder.gpu.kernel()
% for z_c = 1:n_z
%     coder.gpu.constantMemory(z_grid);
%     z = z_grid(z_c);
%     for a_c=1:n_a
%         coder.gpu.constantMemory(a_grid);
%         a = a_grid(a_c);
%         profit = solve_entre(a,z,w,r,lambda,delta,alpha,upsilon);
%         cash(a_c,z_c) = max(w,profit) + (1+r)*a; % cash depends only on (a,z)
%     end
% end
% 
% coder.gpu.kernel()
% for z_c = 1:n_z
%     for a_c = 1:n_a 
% 
%         coder.gpu.constantMemory(cash);
%         cash_is = cash(a_c,z_c) ;
% 
%         for aprime_c = 1 : n_a  % Now introduce a'
% 
%            coder.gpu.constantMemory(a_grid);
%            cons = cash_is - a_grid(aprime_c);   
%            if cons > 0
%                ReturnMatrix(aprime_c,a_c,z_c) = (cons^(1-crra))/(1-crra);
%            else
%                ReturnMatrix(aprime_c,a_c,z_c) = - Inf ;
%            end %end if
%         end %end aprime
%     end %end a
% end %end z
% 
% time_ret = toc;

%% 1.1 the value and policy function

V0     = coder.nullcopy(zeros(n_a,n_z)) ; % Initial guess V0
V      = coder.nullcopy(zeros(n_a,n_z)) ;
Policy = coder.nullcopy(zeros(n_a,n_z)) ;

err  = vfoptions.tolerance+1;
iter = 1;

pi_z_transposed = pi_z';

tic
while err > vfoptions.tolerance

    EV = V0*pi_z_transposed; %EV(a',z)
    
    parfor z_c=1:n_z
        for a_c=1:n_a
            tmpmax = - Inf ;
            maxid = 1 ;
            for aprime_c = 1 : n_a

                entireRHS = ReturnMatrix(aprime_c,a_c,z_c) + beta*EV(aprime_c,z_c);
                if tmpmax < entireRHS 
                    tmpmax = entireRHS ;
                    maxid = aprime_c ;
                end
                V(a_c,z_c)      = tmpmax;
                Policy(a_c,z_c) = maxid;
            end
        end
    end

    % -------------------------- Howard ----------------------------------%
    for h_c = 1 : vfoptions.howards

        EVh = V*pi_z_transposed;
        
        parfor z_c = 1:n_z
            for a_c = 1:n_a
                aprime_opt = Policy(a_c,z_c) ;
                V(a_c,z_c) = ReturnMatrix(aprime_opt,a_c,z_c) + beta*EVh(aprime_opt,z_c) ;
            end
        end
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
time_vfi = toc;

if verbose >= 1
    fprintf('Time return matrix:       %8.6f \n',time_ret);
    fprintf('Time vfi:                 %8.6f \n',time_vfi);
    fprintf('Time return matrix + vfi: %8.6f \n',time_ret+time_vfi);
end

end %end function fun_VFI_cuda

