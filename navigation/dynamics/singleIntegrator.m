function [x_dot] = singleIntegrator(t,x,navigation_params)% xdot = u dynamics

x_goal = navigation_params.x_goal;
rad_from_goal = navigation_params.rad_from_goal;
saturation = navigation_params.saturation;

[~,~,~,lqr_params] = get_params(x);
A = lqr_params.A; B = lqr_params.B;
K = lqr_params.K;

% x1 = x(1,:); 
% x2 = x(2,:);
u = navigation_params.ctrl_multiplier*grad_density_f(x);

%% update x_dot;
if(norm(x-x_goal')<rad_from_goal)
    x_dot = zeros(length(x), 1);
    % LQR Feedback Gain (has overshoot issues)
    %x_dot = A*x + B*K*(x-x_goal');
else
    % Preserve the gradient of the control
    % Normalization method to bound if constraint not satisfied while
    % preserving the direction of the gradient vector
    max_u = max(abs(u));
    if max_u <= saturation % If satisfy friction constraints
        x_dot = u;
    else
        x_dot = u/max_u*saturation;
%    x_dot = 2*tanh(u);
%    x_dot = u;
    end

end
end

