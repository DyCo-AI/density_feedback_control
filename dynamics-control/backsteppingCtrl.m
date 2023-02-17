function [x_bs, u_bs] = backsteppingCtrl(x_0, x_1_ref, xd, alpha_2, beta_2, deltaT, ctrl_multiplier, rad_from_goal, dbl_int_p)
%backsteppingCtrl
%   Performs backstepping on a ref virtual control trajectory (x_1_ref)
%   and the corresponding 0th level state to follow (x_0) given some
%   parameters. This control scheme uses a mixing controller between
%   density feedback control for navigation and LQR for stabilization near
%   the goal or fixed point
%
% Inputs:
%   x_0             : 0th level state (R^(nxN))
%   x_1_ref         : Virtual Control Trajectory or virtual state 
%                       trajectory to track. This is the 1st level ref 
%                       state for backstepping (R^(nx(N-1))
%   x_d             : Desired final goal state in Euclidean space
%   alpha_2         : Tuning parameter for the 1st level density function 
%                       rho_1 = 1/||x||^(2*alpha_2) (R^1)
%   beta_2          : Tuning parameter for the error gain controller
%                       beta_2*(v-k) (R^1)
%   deltaT          : Discrete time step (R^1)
%   step_size       : Control multiplier (R^1)
%   rad_from_goal   : Distance from goal to switch to LQR control (R^1)
%   dbl_int_p       : Structure variable for double integrator parameters
%                       (e.g. LQR Feedback discrete gain matrix)
%
% Outputs:
%   x_bs            : State w/ 0th and 1st level states (i.e includes
%                       backstepping states. (R^(2nxN)), 
%                       Note: These are states and returns the discrete
%                       form trajectory
%   u_bs            : Backstepping control on the 1st level (R^(n x (N-1)))




% TODO(AZ): Create fix size arrays
% TODO(AZ): Make this more compact & neater

% Initialize conditions on x & u w/ backstepping states
x_0 = x_0';
x_1_ref = x_1_ref';

x_bs = [x_0(:,1); x_1_ref(:,1)]; % Typically should initialize to bs states to first virtual control states
u_bs = [];
dim_x = length(x_0(:,1));

% Initialize desired final state
x_f_ref = [xd; zeros(dim_x, 1)];

for i=1:length(x_1_ref)
    % Getting current states and control
    x_0_current = x_bs(1:dim_x, i); % Get current 0th level state (i.e. position states)
    x_1_ref_current = ctrl_multiplier*grad_density_f(x_0_current);
    x_1_ref_current = x_1_ref_current(1:dim_x); % Ensure get only the states
    
    grad_dens = grad_density_f(x_0_current);
    hess_dens = hess_density_f(x_0_current);
    
    % TODO(AZ): Integrate LQR Control when close to goal
    if(norm(x_bs(1:2,end)) <= rad_from_goal)
        %fprintf("Current state: (%0.2f, %0.2f) | Iteration i: %d\n", x_bs(1,end), x_bs(2,end), i);
        
        % Stop control
        %u_bs = [u_bs, zeros(length(u_bs(:,end)), 1)];
        %x_bs = [x_bs, x_bs(:,end)]; % Needs to have control to stop the states but OK for now
        
        % LQR Mixing Control
        u_bs = [u_bs, -dbl_int_p.K_d*(x_bs(:,i)-x_f_ref)]; % u[k] = -K_d*x[k]
        x_bs = [x_bs, (dbl_int_p.A_d*x_bs(:,i) + dbl_int_p.B_d*u_bs(:,i))]; % x[k+1] = A_bs*x[k] + B_bs*u[k]
    else
        % Find backstepping controller
        u_bs = [u_bs, ((x_bs(dim_x+1:end,end)-x_1_ref_current)' * (x_bs(dim_x+1:end,end)-x_1_ref_current) * ... % (v-k)^T*(v-k)
            grad_dens(1:length(x_0_current)))/((2*alpha_2 - 2)*density_f(x_0_current)) + ... % grad_x(g*rho)
            hess_dens(1:length(x_0_current), 1:length(x_0_current))*x_bs(dim_x+1:end,end) - ... % (doh k)/(doh x)*(f+g*v)
            beta_2*(x_bs(dim_x+1:end,end)-x_1_ref_current)]; % Beta_2*(v-k)
        x_bs = [x_bs, [x_bs(1:dim_x,i) + deltaT*x_bs(dim_x+1:end, i);
            0;
            0]];
        x_bs(dim_x+1:end,end) = x_bs(dim_x+1:end,i) + deltaT*u_bs(:,end);
    end
    
end


end