function [x_euler, u_euler] = forwardEuler(dynamics,navigation_params)

%forwardEuler
%   Forward euler integration to propagate dynamics
% Inputs:
%   dynamics            : Function handle to use as dynamics
%   navigation_params   : params with navaigation paramters
%
% Outputs:
%   x_euler             : Forward integrated states (R^(nxN))
%   u_euler             : Control input trajectory (R^(nx(N-1)))

x0 = navigation_params.x_ini;
[~,euler_params] = get_params(x0);
N = euler_params.n_steps;
dT = euler_params.step_size;

u_euler = zeros(length(x0),N);
x_euler = zeros(length(x0),N);

x_euler(:,1) = x0;
% wrap around SE2 torous for q1 and q2
if(x_euler(1,1) > 2*pi)
    disp('wrap:0');
    x_euler(1,1) = x_euler(1,1) - 2*pi; 
end
if(x_euler(2,1) > 2*pi)
    disp('wrap:0')
    x_euler(2,1) = x_euler(2,1) - 2*pi; 
end
if(x_euler(1,1) < 0)
    disp('wrap:0')
    x_euler(1,1) = 2*pi + x_euler(1,1);
end
if(x_euler(1,1) < 0)
    disp('wrap:0')
    x_euler(2,1) = 2*pi + x_euler(2,1);
end


for n=1:N-1 % For loop, sets next t,y values
    x = x_euler(:,n); t = 0; % LTI dynamics
    u_euler(:,n) = dynamics(t, x, navigation_params);
    x_euler(:,n+1) = x + dT*u_euler(:,n);

    % wrap around SE2 torous for q1 and q2
    if(x_euler(1,n+1) > 2*pi)
        disp('wrap:'); disp(n);
        x_euler(1,n+1) = x_euler(1,n+1) - 2*pi; 
    end
    if(x_euler(2,n+1) > 2*pi)
        disp('wrap:'); disp(n);
        x_euler(2,n+1) = x_euler(2,n+1) - 2*pi;
    end
    if(x_euler(1,n+1) < 0)
        disp('wrap:'); disp(n);
        x_euler(1,n+1) = 2*pi + x_euler(1,n+1);
    end
    if(x_euler(2,n+1) < 0)
        disp('wrap:'); disp(n);
        x_euler(2,n+1) = 2*pi + x_euler(2,n+1);
    end       
end
end