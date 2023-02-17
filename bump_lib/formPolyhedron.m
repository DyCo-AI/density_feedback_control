function bump = formPolyhedron(r1, r2, c, x, n, vpa_enable)
%formStep
% Forms polyhedron by intersection of hyperplane / step functions
%
% Inputs:
%   r1          : Positive definite value of the length to the
%                   boundary of the zero level set (R^(1xm))
%   r2          : Positive definite value of boundary to 1-level set 
%                   r2 = r1 + epsilon (R^(1xm)) -> f_step(r2) = 1
%   c           : "Center" of step to origin, which get mapped to scalar
%                   value s.t. perpendicular distance from origin to center
%                   of step function (R^(nxm))
%   x           : Symbolic variable of R^(nx1) (n determines dimension of bump)
%   n           : Normal vector of hyperplane (R^(nxm)) (i.e. direction of the
%                   1-level set)
%   vpa_enable  : Boolean to enable VPA precision w/ digit 10
%
% Outputs:
%   bump        : Symbolic expression of polyhedron bump

% TODO: Use compose s.t. mathematically cleaner...
% currently having trouble w/ replicating results from compose


if nargin < 6
    vpa_enable = false;
end

% Check # hyperplane / step / m > 3 -> Closed and convex
num_step = size(c, 2);
if num_step < 3
    error('Number of hyperplanes must be > 3 to form a closed shaped');
end

% Form a bounded convex shape by intersections of hyperplane
bump = 1; % Initialize
for i = 1:num_step
    bump = bump*formStep(r1(:,i), r2(:,i), c(:,i), x, n(:,i), vpa_enable);
end

bump = -bump + 1;

end