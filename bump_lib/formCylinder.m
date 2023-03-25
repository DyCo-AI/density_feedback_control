function bump = formCylinder(c, r, r_s, x, vpa_enable)
%formCylinder
% Forms cylinder function in R^3 using MATLAB symbolic variable x
%
% Inputs:
%   c           : Center of bump function (R^3)
%   r           : Radius of cylinder (R^1)
%   r_s         : Sensing radius (R^1)
%   x           : Symbolic variable of R^3
%   vpa_enable  : Boolean to enable VPA precision w/ digit 10
%
% Outputs:
%   bump        : Symbolic expression of cylindrical inverse bump function

% TODO: Figure out how to use compose s.t. mathematically cleaner...
if nargin < 6
    vpa_enable = false;
end

cx = c(1);
cy = c(2);
cz = c(3);
r1 = r;
r2 = r_s;

shape = (sqrt((x(1)-cx)^2 + (x(2)-cz)^2))^2 - r^2;

shift = 1; % to get rid of random zeros | needs to be nonzeros
% h =  shape + shift;
h = shape;
m = h/(r2^2-r1^2);

f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));


% Normalize the symmetric step function
g = f/(f+f_shift);
bump = g; % Emphasis that g is bump function

if vpa_enable
    bump = vpa(bump,5);
end


end