function yi = interp1_scal(x, y, xi)
% Custom linear interpolation with extrapolation, matching interp1(x,y,xi,'linear','extrap')
% x and y are vectors of the same length, x strictly increasing

coder.gpu.kernelfun;

n = length(x);

% Handle exact match with first or last point (avoid extrapolation)
if xi <= x(1)
    j = 1;       % extrapolate using [x(1), x(2)]
elseif xi >= x(n)
    j = n - 1;   % extrapolate using [x(n-1), x(n)]
else
    % xi is in-bounds; locate correct interval
    j = locate(x, xi);
end

% Enforce valid index range
j = max(min(j, n - 1), 1);

% Avoid divide-by-zero (can happen if x has duplicate points)
dx = x(j+1) - x(j);
if dx == 0
    slope = 0;
else
    slope = (y(j+1) - y(j)) / dx;
end

yi = y(j) + slope * (xi - x(j));

end


% function jl = locate(xx, x)
% % Return j such that xx(j) <= x < xx(j+1), for strictly increasing xx
% coder.gpu.kernelfun;
% 
% n = length(xx);
% if x < xx(1)
%     jl = 0;
% elseif x > xx(n)
%     jl = n;
% else
%     jl = 1;
%     ju = n;
%     while (ju - jl > 1)
%         jm = floor((ju + jl) / 2);
%         if x >= xx(jm)
%             jl = jm;
%         else
%             ju = jm;
%         end
%     end
% end
% end
% 
