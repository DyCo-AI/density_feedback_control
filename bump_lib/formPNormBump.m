function bump = formPNormBump(r1, r2, c, x, p, vpa_enable, A)
%formCircleBump
% Forms circular bump function of R^n based off obstacle radius (r1), sensing
% radius (epsilon = r2-r1), center, and MATLAB symbolic variable x
%
% Inputs:
%   r1          : Positive value of radius of obstacle (R^1)
%   r2          : Positive value of obstacle radius + sensing radius
%                   r2 = r1 + epsilon (R^1)
%   c           : Center of bump function (R^n)
%   x           : Symbolic variable of R^n (n determines dimension of bump)
%   p           : Scalar to choose the p-norm
%   vpa_enable  : Boolean to enable VPA precision w/ digit 10
%   A           : Diffeomorphic mapping (R^(nxn))
%
% Outputs:
%   bump        : Symbolic expression of bump

% TODO: Figure out how to use compose s.t. mathematically cleaner...
% currently having trouble w/ replicating results from compose
if nargin < 5
    p = 2; % Default 2-norm
end
if nargin < 6
    vpa_enable = false;
end
if (nargin < 7)
    A = eye(size(x,1));
end




% Linear change of coordinates and symmetric w/ norm(x)^2
m = (norm(A*(x-c), p)^p-r1^p)/(r2^p-r1^p);

f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));

% Normalize the symmetric step function
g = f/(f+f_shift);
bump = g; % Emphasis that g is bump function

if vpa_enable
    bump = vpa(bump,5);
end


end