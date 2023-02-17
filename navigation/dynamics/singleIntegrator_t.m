function [x_dot] = singleIntegrator_t(t,x,params)

% xdot = u dynamics
% x1 = x(1,:); 
% x2 = x(2,:);
u = params.ctrl_multiplier*grad_density_f(x,t);

%% update x_dot;
if(norm(x-params.x_goal)<params.rad_from_goal)
    %x_dot = zeros(length(x), 1);
    % LQR Feedback Gain
    x_dot = (params.A - params.B*params.K)*x + params.B*params.K*params.x_goal;
else
    %x_dot = u(1:length(x)); % Take gradient of only states x
    max_u = max(abs(u));
    if max_u <= params.mu*params.mg % If satisfy friction constraints
        x_dot = u;
    else
        x_dot = u/max_u*(params.mu*params.mg);
%    x_dot = 2*tanh(u);
%    x_dot = u;
    end
end

end

