function [xf, fval, exitflag] = brent_min(funfcn, ax, bx, tol, maxfun, maxiter)
%#codegen
% brent_min: Single-variable bounded function minimization (Brent's method)
% without derivatives.
%
% INPUTS
%   funfcn  : function handle to the scalar function to be minimized
%   ax      : lower bound of the interval
%   bx      : upper bound of the interval
%   tol     : tolerance criterion for X
%   maxfun  : maximum number of function evaluations
%   maxiter : maximum number of iterations
%
% OUTPUTS
%   xf       : minimizing x
%   fval     : function value at xf
%   exitflag : 1 if converged, 0 if maxfun or maxiter reached

coder.inline('always');

funccount = 0;
iter      = 0;

% Assume convergence unless we hit limits
exitflag = 1;

% Constants
seps = sqrt(eps);
c    = 0.5 * (3.0 - sqrt(5.0));  % 0.381966011250105...

% Initial bracketing
a = ax;
b = bx;

v  = a + c * (b - a);
w  = v;
xf = v;

d = 0.0;
e = 0.0;

x  = xf;
fx = funfcn(x);
funccount = funccount + 1;

fv = fx;
fw = fx;

xm   = 0.5 * (a + b);
tol1 = seps * abs(xf) + tol / 3.0;
tol2 = 2.0 * tol1;

% Main loop
while abs(xf - xm) > (tol2 - 0.5 * (b - a))

    gs = 1;  % flag for golden-section step

    % Attempt parabolic step if possible
    if abs(e) > tol1
        gs = 0;

        r = (xf - w) * (fx - fv);
        q = (xf - v) * (fx - fw);
        p = (xf - v) * q - (xf - w) * r;
        q = 2.0 * (q - r);

        if q > 0.0
            p = -p;
        end
        q = abs(q);

        r = e;
        e = d;

        % Check whether the parabolic step is acceptable
        if (abs(p) < abs(0.5 * q * r)) && (p > q * (a - xf)) && (p < q * (b - xf))
            % Parabolic interpolation step
            d = p / q;
            x = xf + d;

            % f must not be evaluated too close to the interval endpoints
            if ((x - a) < tol2) || ((b - x) < tol2)
                if xm >= xf
                    d =  tol1;
                else
                    d = -tol1;
                end
            end
        else
            % Reject parabola, do golden-section step
            gs = 1;
        end
    end

    if gs ~= 0
        % Golden-section step
        if xf >= xm
            e = a - xf;
        else
            e = b - xf;
        end
        d = c * e;
    end

    % The function must not be evaluated too close to xf
    if d >= 0
        x = xf + max(d,  tol1);
    else
        x = xf + min(d, -tol1);
    end

    fu = funfcn(x);
    funccount = funccount + 1;
    iter      = iter + 1;

    % Update a, b, v, w, x, xm, tol1, tol2
    if fu <= fx
        if x >= xf
            a = xf;
        else
            b = xf;
        end
        v  = w;  fv = fw;
        w  = xf; fw = fx;
        xf = x;  fx = fu;
    else
        if x < xf
            a = x;
        else
            b = x;
        end

        if (fu <= fw) || (w == xf)
            v  = w;  fv = fw;
            w  = x;  fw = fu;
        elseif (fu <= fv) || (v == xf) || (v == w)
            v  = x;  fv = fu;
        end
    end

    xm   = 0.5 * (a + b);
    tol1 = seps * abs(xf) + tol / 3.0;
    tol2 = 2.0 * tol1;

    % Stopping due to limits
    if (funccount >= maxfun) || (iter >= maxiter)
        exitflag = 0;
        fval     = fx;
        return
    end
end

fval = fx;

end
