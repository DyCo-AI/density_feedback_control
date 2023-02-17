%% Obstacle Avoidance using single integrator dynamics
clc
clear
close all
%% Problem setup
mkdir('functions');
mkdir('utils');
mkdir('dynamics-control');
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('bump_lib');

tic
% Navigation Parameters
% Start position
nav_p.x0 = [0;0;0;0]; % Initial Condition
nav_p.xd = [12;0.1;0;0]; % Final condition
nav_p.p = 2; % Generate obstacle set off p-norm

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 1;

% Gains on double integrator control
nav_p.kp = 10; % Gain on grad
nav_p.kd = 1;
nav_p.xddot_d = [0;0];

% Ego Vehicle Size
nav_p.ego_r = 0.0; % Diameter of ego
sample_rate = 50; % Every sample_rate to plot trajectory of vehicle

% General function: f(x) = ke^(-a/(1-b(x-c)^2)) + d
% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
nav_p.r1 = 0.5;
nav_p.r2 = 1.5;

% Obstancle centers (e.g. c1 and c2) | Domain R^n
num_obs = 4;
nav_p.c1 = [2; 0];
nav_p.c2 = [8; 0];
nav_p.c3 = [5; 2];
nav_p.c4 = [5;-2];
nav_p.c0 = 0; % Add weaker condition -> unsafe set

theta = 0*pi/180; % [Radian] CounterClockwise | Rotate after
stretch = [1;1];
gamma = 0*pi/180; % [rad] CCW | Post rotate

% Function: g(x):= 1/||x||^alpha
nav_p.alpha = 0.2; % Best value 0.2

% Euler Parameters
N = 15000; %timesteps
deltaT = 0.01;
ctrl_multiplier = 1; % Parameter to change | kp and ctrl_mult the same

% Rantzer Check Parameters
check_rantzer = false;
rantzer_graph = false;

% Optimize function handle generation
optimize = false; % Decision to optimize function generation | false for faster generation time
vpa_enable = true;

if vpa_enable
    optimize = false; % Set optimize also false s.t. symbolic tool generation is fast / manageable
end

% Save figures
save_fig = true;

% Save state and ctrl trajectory in csv file
export_to_csv = true;
state_filename = 'states_traj_obs.csv';
ctrl_filename = 'ctrl_traj_obs.csv';

%% Density Function Formulation
% Create function of density and bump function
syms x [2,1] real % Spatial states

[A, A_inv] = transformationMatrix(theta, stretch, 2);

% Obstacle function
% bump = formPNormBump(nav_p.r1,nav_p.r2, nav_p.c1, x, nav_p.p, true, A_inv);
bump = formPNormBump(nav_p.r1,nav_p.r2, nav_p.c1, x, nav_p.p, true, A_inv)*formPNormBump(nav_p.r1,nav_p.r2, nav_p.c2, x, nav_p.p, true, A_inv)*...
        formPNormBump(nav_p.r1,nav_p.r2, nav_p.c3, x, nav_p.p, true, A_inv)*formPNormBump(nav_p.r1,nav_p.r2, nav_p.c4, x, nav_p.p, true, A_inv) ...
        + nav_p.c0;
%bump = formFastInvBump(x, nav_p, num_obs, A_inv, true) + nav_p.c0;


g = 1/norm(x-nav_p.xd(1:length(x)))^(2*nav_p.alpha); % rho
%sigma = 2.5;
%g = 5*exp(-norm(x)^2/(2*sigma^2))+1; % rho

% Density function
density = g*bump;

grad_density = gradient(density, x);
%hess_density = hessian(density, x);
% Create function handles
matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x}, 'Optimize', optimize);
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x}, 'Optimize', optimize);
%matlabFunction(hess_density, 'File', 'functions/hess_density_f', 'Vars', {x}, 'Optimize', optimize);
toc

%% Plotting Density & Rantzer Conditions
x_vec = -10:0.25:10; y_vec = x_vec;
[X,Y] = meshgrid(x_vec,y_vec);
rho_val = zeros(size(X));
grad_rho_x1 = zeros(size(X));
grad_rho_x2 = zeros(size(Y));

for i=1:length(x_vec)
    for j = 1:length(y_vec)
        rho_val(i,j) = density_f([x_vec(j);y_vec(i)]);
        grad_rho = grad_density_f([x_vec(j);y_vec(i)]);
        grad_rho_x1(i,j) = grad_rho(1);
        grad_rho_x2(i,j) = grad_rho(2);
    end
end

surf(X,Y,rho_val, 'FaceAlpha',0.65, 'EdgeColor', 'none')
colormap jet
view(90,0)
title("Density Function")
xlabel("x_1")
ylabel("x_2")
zlim([-1 3])

figure()
quiverInLogScale(X, Y, grad_rho_x1, grad_rho_x2); % Add 3 for scaling
title("Log Scale Gradient of Density Function");
hold on;

% Check Rantzer's Condition Graphically | div (rho u)
if check_rantzer || rantzer_graph
    [rantzer_cond, pos_not_rantz_cond, rantzer_graph] = rantzer_check(check_rantzer, rantzer_graph);
end

%% Form system matrices and LQR Gain
[~, dbl_int_p, dyn_p] = generateStateSpace(deltaT, x);

%% using Euler method
           
% [x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, ...
%                         @doubleIntegrator, dbl_int_p);
[x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, ...
                        @eulerLagrangeDynamics, dyn_p);
toc
%% Plot
obsColor = [.7 .7 .7]; % Obstacle color -> Grey

figure()


if nav_p.ego_r == 0
    state_traj = plot(x_euler(1:end,1),x_euler(1:end,2),'b-', 'LineWidth', 2); hold on;
else
    state_traj = plot(x_euler(1:sample_rate:end,1),x_euler(1:sample_rate:end,2),'bo', 'LineWidth', 2); hold on;
end
plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
plot(nav_p.xd(1), nav_p.xd(2), 'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
dummy_marker = plot(NaN,NaN, 'o','MarkerSize', 10, 'MarkerEdgeColor',...
            'black', 'MarkerFaceColor',obsColor, 'LineWidth', 1.5); % For legend as rectangular object can't be defined as a legend

gamma = (0:100-1)*(2*pi/100);
points = nav_p.c1 + A*[nav_p.r1*cos(gamma);nav_p.r1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
points = nav_p.c2 + A*[nav_p.r1*cos(gamma);nav_p.r1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
points = nav_p.c3 + A*[nav_p.r1*cos(gamma);nav_p.r1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
points = nav_p.c4 + A*[nav_p.r1*cos(gamma);nav_p.r1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;


xlim([0,10]);
ylim([-5,5]);
axis square

lgd = legend('Trajectory', 'Start','Goal','Obstacles');
lgd.FontSize = 14;
set(lgd,'interpreter','latex')
%title('Euclidean Space Trajectory');
xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

% Convert marker size to radius
ax = gca();
if nav_p.ego_r > 0
    convertMarkerSize(nav_p.ego_r*2, state_traj, ax);
end

t = 1:size(x_euler,1);
figure()
plot(t,x_euler)
%title("State trajectory")
xlabel("Iteration", 'FontSize', 20)
ylabel("State", 'FontSize', 20)
state_lgd = legend("x1", "x2", "x3", "x4");
state_lgd.FontSize = 14;

t = 1:size(u_euler,1);
figure()
plot(t,u_euler)
%title("Control trajectory")
xlabel("Iteration", 'FontSize', 20)
ylabel("Control", 'FontSize', 20)
ctrl_lgd = legend("u1", "u2");
ctrl_lgd.FontSize = 14;

%% Export to CSV File
if export_to_csv
    writematrix(x_euler, state_filename);
    writematrix(u_euler, ctrl_filename);
end


%% helper functions
function [min_dist, index, diff_vector] = minDistFromC(c, x)
% minDistFromC determines the minimum distance of (x-c) where x is a 
% trajectory of vectors and c is a point
%
% Input:
%   c           : Vector value point in R^(1xn)
%   x           : Trajectory of states in R^(mxn) where m is traj. length
%
% Output:
%   min_dist    : Minimum distance (or norm) from the trajectory of states x
%                   and point c
%   index       : Index of the minimum distance
%   diff_vector : Minimum vector difference between point c 
for i=1:length(x)
    diff(i) = norm(x(i,:) - c);
end
[min_dist, index] = min(diff);
diff_vector = x(index,:) - c;
end


% Currently not being used (used for ode45)
function [value1, value2, isterminal, direction] = detect_NaN(T, Y)
% collect y and t values when NaN occurs during ODE45
value1      = (isnan(Y(1)));
value2      = (isnan(Y(2)));
isterminal = 1;   % Stop the integration
direction  = 0;
end