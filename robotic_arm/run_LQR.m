clc; clear; close all;

%% run LQR
params.system = 1; % 1 - Acrobot
params.predHorizon = 5;
params.l1 = 1;
params.l2 = 1;

if(params.system == 1)
    % ode45 needs a row vector for x0 and xd
    xd = [pi 0 0 0];
    x0 = [0 0.003 0 0];
    
    x = sym('x',[4;1]); u = sym('u',[2;1]);
    f = @dynamics_planarRR;
    A = double(subs(jacobian(f(0,x,zeros(size(u))),x),x,xd')); 
    B = double(subs(jacobian(f(0,xd',u),u),u,zeros(size(u))));
    
    Q = diag([10 10 1 1]);
    R = 1;
    K = lqr(A,B,Q,R);
    u = @(x) -K*(x-xd');
    
    tspan = 0:0.01:5;
    [t,x] = ode45(@(t,x)f(t,x,u(x)), tspan, x0);
    
    % animate
    animate_planarRR(t,x,params)
end