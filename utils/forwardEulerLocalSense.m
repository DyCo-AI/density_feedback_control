function [x_euler, u_euler] = forwardEulerLocalSense(p, N, deltaT, ctrl_multiplier, dynamics, single_int_p, local_sense, obs_name, x)
%forwardEuler
%   Forward euler integration to propagate dynamics
%
% Inputs:
%   p                   : Parameters specifically containing init
%                           conditions x0 (R^n), goal state xd (R^n) & 
%                           radius from goal (R^1)
%   N                   : Number of forward integration
%   deltaT              : Discrete step size (R^n)
%   ctrl_multiplier     : Multiplier on control (R^n)
%   dynamics            : Function handle to use as dynamics
%   single_int_p        : Single Integrator parameter. More specifically,
%                           the A, B system matrices and the LQR Feedback 
%                           Gain close to goal
%   local_sense         : Boolean to enable local sensing mode
%   obs_name            : List of obstacle names (R^o) (NECESSARY TO USE)
%   x                   : Symbolic variable to update density if local sensing
%                           (R^n)
%
% Outputs:
%   x_euler             : Forward integrated states (R^(nx(N+1)))
%   u_euler             : Control input trajectory (R^(nx(N)))

if nargin < 7
    local_sense = false;
end

if nargin < 8
    obs_name = [];
end

if nargin < 9
    syms x [2,1] real
end

x_euler = zeros(size(p.x0,1), N+1);
u_euler = zeros(size(p.x0,1), N);
x_euler(:,1) = p.x0;

curr_obs_loc = [];

for n=1:N % For loop, sets next t,y values
    
    u_euler(:,n) = dynamics(n*deltaT, x_euler(:,n), ctrl_multiplier, ...
                            p, single_int_p);
    x_euler(:, n+1) = x_euler(:,n) + deltaT*u_euler(:,n);
    
    if local_sense
        [obstacle_bool, ~, close_obs_list] = localSensing(p, ...
            x_euler(:,n+1), obs_name, curr_obs_loc, 0);
        
        % obstacle_bool if need to update curr_obs_loc list since new
        % obstacle that is within local sensing radius
        if obstacle_bool
            curr_obs_loc = close_obs_list;
            updateDensity(x, p, curr_obs_loc); % TODO: Figure out how to incorporate fastinvbump w/ local sensing here... maybe change params?
        end
    end
end

x_euler = x_euler';
u_euler = u_euler';

end