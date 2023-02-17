function [x_dot, u] = singleIntegrator(t,x,ctrl_multiplier, p, single_int_p, dens_bool)
%singleIntegrator
%   Propagates the kinematic model (i.e. single integrator system) with a
%   control and changes to a LQR control given a radius from goal
% Inputs:
%   t                   : Discrete time
%   x                   : Current state of the system (R^n)
%   ctrl_multiplier     : Scaling value to increase control value input
%   p                   : Navigation parameter containing xd (Goal state)
%                           and radius from goal rad_from_goal (R^1)
%   single_int_p        : Single Integrator parameters. Specifically the
%                           dynamical system matrix A, B and FB optimal
%                           gain K from the Riccatti equation
%   dens_bool           : Boolean to enable density functions vs navigation
%                           functions
% Outputs
%   x_dot               : The velocity state or control input (x_dot = u)
%   u                   : Control input
% TODO(AZ): Need to think about how to generalize this formulation... right
% now it seems okay to have different dynamics but backstepping
% formulation needs to be more general

if nargin < 7
    dens_bool = true;
end


% Parameters
rad_from_goal = p.rad_from_goal;
xd = p.xd;
saturation = 2;


if dens_bool
    u = ctrl_multiplier*grad_density_f(x);
else
    u = -ctrl_multiplier*grad_phi_f(x);
end

[max_u, ~] = max(abs(u));
if max_u >= saturation
    u = u/max_u*saturation;
end
    

% Switch control
if(norm(x-xd)<rad_from_goal)
    %x_dot = zeros(length(x), 1);
    
    % LQR Feedback Gain
    u = -single_int_p.K*(x-xd);
    x_dot = single_int_p.A*x+single_int_p.B*u;
else
    x_dot = u; % Since A = 0 -> x_dot = A*x + u -> x_dot = u
end

end

