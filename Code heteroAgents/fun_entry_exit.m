function [exit_E_to_W, entry_W_to_E, T] = fun_entry_exit(pi_z, StatDist, Policy, pol_e, n_a, n_z)
% Computes entry and exit rates using scalar accumulators
% Transition counts:
% T(1,1): W→W
% T(1,2): W→E
% T(2,1): E→W
% T(2,2): E→E

T = zeros(2,2);

for a_c = 1:n_a
    for z_c = 1:n_z
        aprime_c = Policy(a_c, z_c);
        occ_now = pol_e(a_c, z_c);       % 0=worker, 1=entrepreneur
        o = 1 + (occ_now == 1);          % 1=worker, 2=entrepreneur

        for zprime_c = 1:n_z
            prob = pi_z(z_c, zprime_c);
            weight = StatDist(a_c, z_c);
            occ_next = pol_e(aprime_c, zprime_c);
            o_prime = 1 + (occ_next == 1);
            T(o, o_prime) = T(o, o_prime) + prob * weight;
        end
    end
end

exit_E_to_W   = T(2,1) / max(sum(T(2,:)), eps);
entry_W_to_E  = T(1,2) / max(sum(T(1,:)), eps);

end

