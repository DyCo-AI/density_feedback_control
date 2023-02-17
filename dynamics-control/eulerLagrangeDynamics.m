function [x_dot, u] = eulerLagrangeDynamics(t,x,ctrl_multiplier, p, dyn_p, dens_bool)
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
%   dynamics_p          : General dynamics parameter. Specifically the
%                           dynamical system matrix mass (M(q)) and
%                           nonlinear terms (H(q,qdot))
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
dim_x = dyn_p.x_dim;
dim_u = dyn_p.u_dim;

% Get spatial state variables
% ** Assumes that # of spatial states and time derivative states are equal **
spatial_x = x(1:dim_x/2);
spatial_x_dim = dim_x/2;
x_deriv = x(dim_x/2+1:end);


e = spatial_x - p.xd(1:spatial_x_dim);
edot = x_deriv - p.xd(dim_x/2+1:end);
% Checks which control to apply
if dens_bool
    grad_density_vals = ctrl_multiplier*grad_density_f(spatial_x); % gradient of error?
else
    grad_density_vals = -ctrl_multiplier*grad_phi_f(spatial_x);
end


% TODO: Fix H as just static
u_rho = dyn_p.M*p.xddot_d + dyn_p.C*x_deriv + dyn_p.G + dyn_p.M*(p.kp*grad_density_vals - p.kd*edot);
% u_rho = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
u = u_rho;

% Saturation
[max_u, ~] = max(abs(u));
if max_u <= saturation || ~saturation_bool
    x_dot = [x_deriv;
                dyn_p.M\(-dyn_p.C*x_deriv - dyn_p.G + dyn_p.B_hat*u)];
%     x_dot = dyn_p.A*x + dyn_p.B*u;
else
    % fprintf("Saturation\n");
    u = u/max_u*saturation;
%     x_dot = dyn_p.A*x + dyn_p.B*u;
    x_dot = [x_deriv;
                dyn_p.M\(-dyn_p.C*x_deriv - dyn_p.G + dyn_p.B_hat*u)];
end
    

% Switch control
% TODO: LQR OSCILLATING MAYBE BC DISCRETIZATION
% TODO: Do LQR for euler lagrange systems somehow w/o A and B matrices
if(norm(x(1:spatial_x_dim)-xd(1:spatial_x_dim))<rad_from_goal)
    % fprintf("Switch Control\n");
    u = zeros(length(dim_u), 1);
    x_dot = zeros(dim_x,1);
    % LQR Feedback Gain
%     u = -dyn_p.K*(x-xd);
%     x_dot = dyn_p.A*x + dyn_p.B*u;
end

end

