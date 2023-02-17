function step = formStep(r1, r2, c, x, n, vpa_enable)
%formStep
% Forms step function of R^n based off "center" and obstacle radius (c+r1),
% sensing radius (epsilon = s-r), and MATLAB symbolic variable x
%
% Inputs:
%   r1          : Positive definite value of the length to the
%                   boundary of the zero level set (R^1)
%   r2          : Positive definite value of boundary to 1-level set 
%                   r2 = r1 + epsilon (R^1) -> f_step(r2) = 1
%   c           : "Center" of step to origin, which get mapped to scalar
%                   value s.t. perpendicular distance from origin to center
%                   of step function R^(nx1)
%   x           : Symbolic variable of R^(nx1)
%   n           : Normal vector of hyperplane (R^n) (i.e. direction of the
%                   1-level set)
%   vpa_enable  : Boolean to enable VPA precision w/ digit 10
%
% Outputs:
%   bump        : Symbolic expression of bump

% TODO: Use compose s.t. mathematically cleaner...
% currently having trouble w/ replicating results from compose

if nargin < 6
    vpa_enable = false;
end


% To get bounds r1 and r2 -> sqrt(r1) & sqrt(r2)
r1 = sqrt(r1);
r2 = sqrt(r2);
c = sum(c);

% Linear change of coordinates and symmetric w/ norm(x)^2
m = ((n'*x-c)-r1^2)/(r2^2-r1^2); % c is the perpendicular distance of center of step to origin
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m<=0, 0, exp(-1/(1-m)));

% Normalize the symmetric step function
g = f/(f+f_shift);
step = g; % Emphasis that g is bump function

if vpa_enable
    step = vpa(step,5);
end

end