% Fast linear interpolation routine
% Usage:
% yi = interp1_scal(x,y,xi)
% where x and y are column vectors with n elements, xi is a scalar and yi
% is a scalar
% Input Arguments
% x - Sample points
%   column vector
% Y - Sample data
%   column vector
% xi - Query point
%   scalar
function yi = interp1_scal(x,y,xi)
coder.gpu.kernelfun;

n = size(x,1);

j = locate(x,xi);
j = max(min(j,n-1),1);

slope = (y(j+1)-y(j))/(x(j+1)-x(j));
yi = y(j)+(xi-x(j))*slope;

end %end function interp1_scal


