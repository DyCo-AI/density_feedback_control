function [x_dot, u] = doubleIntegrator(t,x,ctrl_multiplier, p, dbl_int_p, dens_bool)
%singleIntegrator
%   Propagates the dynamic model (i.e. double integrator system) with a
%   control and changes to a LQR control given a radius from goal.
%   Assumes canonical state space form
% Inputs:
%   t                   : Discrete time
%   x                   : Current state of the system (R^(nx1))
%   ctrl_multiplier     : Scaling value to increase control value input
%   p                   : Navigation parameter containing xd (Goal state)
%                           and radius from goal rad_from_goal (R^1) for
%                           switch control
%   dbl_int_p           : Double Integrator parameters. Specifically the
%                           dynamical system matrix A, B and FB optimal
%                           gain K from the Riccatti equation
%   dens_bool           : Boolean to enable density functions vs navigation
%                           functions
% Outputs
%   x_dot               : The derivative states (R^(nx1))
%   u                   : Control input (R^(mx1))

% TODO(AZ): Need to think about how to generalize this formulation... right


if nargin < 7
    dens_bool = true;
end


% Parameters
rad_from_goal = p.rad_from_goal;
xd = p.xd;
kp = p.kp;
kd = p.kd;

saturation_bool = true;
saturation = 2;
dim_x = size(dbl_int_p.A,1);
dim_u = dbl_int_p.u_dim;

% Get spatial state variables
% ** Assumes that # of spatial states and time derivative states are equal **
spatial_x = x(1:dim_x/2);
spatial_x_dim = dim_x/2;


% Checks which control to apply
if dens_bool
    grad_density_vals = ctrl_multiplier*grad_density_f(spatial_x);
else
    grad_density_vals = -ctrl_multiplier*grad_phi_f(spatial_x);
end

u_rho = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
u = u_rho;

% Saturation
[max_u, ~] = max(abs(u));
if max_u <= saturation || ~saturation_bool
    x_dot = dbl_int_p.A*x + dbl_int_p.B*u;
else
    % fprintf("Saturation\n");
    u = u/max_u*saturation;
    x_dot = dbl_int_p.A*x + dbl_int_p.B*u;
end
    

% Switch control
% TODO: LQR OSCILLATING MAYBE BC DISCRETIZATION
if(norm(x(1:spatial_x_dim)-xd(1:spatial_x_dim))<rad_from_goal)
    % fprintf("Switch Control\n");
%     u = zeros(length(dim_u), 1);
%     x_dot = zeros(dim_x,1);
    % LQR Feedback Gain
    u = -dbl_int_p.K*(x-xd);
    x_dot = dbl_int_p.A*x + dbl_int_p.B*u;
end

end

