function [x_euler, u_euler, x_dot] = forwardEuler(p, N, deltaT, ctrl_multiplier, dynamics, dyn_p, dens_bool)
% TODO(AZ): Create fixed size array instead of using dynamic array
%forwardEuler
%   Forward euler integration to propagate dynamics. Returns x
%
% Inputs:
%   p                   : Parameters specifically containing init
%                           conditions x0 (R^n), goal state xd (R^n) & 
%                           radius from goal (R^1)
%   N                   : Number of forward integration
%   deltaT              : Discrete step size (R^n)
%   ctrl_multiplier     : Multiplier on control (R^n)
%   dynamics            : Function handle to use as dynamics
%   dyn_p               : Dynamic system parameter. More specifically,
%                           the A, B system matrices, the LQR Feedback 
%                           Gain close to goal, and dimension of control
%
% Outputs:
%   x_euler             : Forward integrated states (R^(nx(N+1)))
%   u_euler             : Control input trajectory (R^(nx(N)))
%   x_dot               : State derivative trajectory (R^(nxN))

if nargin <  7
    dens_bool = true;
end

x_euler = zeros(size(p.x0,1), N+1);
x_dot = zeros(size(p.x0,1), N);
x_euler(:,1) = p.x0;

u_euler = zeros(dyn_p.u_dim,N);

for n=1:N % For loop, sets next t,y values
    [x_dot(:, n), u_euler(:,n)] = dynamics(n*deltaT, x_euler(:,n), ctrl_multiplier, ...
                            p, dyn_p, dens_bool);
    x_euler(:, n+1) = x_euler(:,n) + deltaT*x_dot(:,n);
    
end

x_euler = x_euler';
x_dot = x_dot';
u_euler = u_euler';

end