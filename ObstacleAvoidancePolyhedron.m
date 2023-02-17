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

% Parameter setup

% Start position
nav_p.x0 = [-5;1]; % Start position
nav_p.xd = [0;5]; % Goal position / Desired position

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 1.0; % Original value: 3.0

sample_rate = 50; % Every sample_rate to plot trajectory of vehicle

% General function: f(x) = ke^(-a/(1-b(x-c)^2)) + d
% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
nav_p.r1 = 1; % Change radius: increase r -> increase radius
            % If parameter d, d >= 1
nav_p.r2 = 1.5;

nav_p.p = 2; % p-norm bump

% Obstancle centers (e.g. c1 and c2) | Domain R^n
num_obs = 5;
nav_p.c1 = [-3; 0]; % [-3; 1]
nav_p.c2 = [3; 0]; % [-3; 1.25]
nav_p.c3 = [0; 3]; % [-3; 1.5]
nav_p.c4 = [0;-3]; % [-3; 1.75]
nav_p.c5 = [-3;2]; % [-3; 2]

nav_p.ub_c1 = [0;-3]; % "Center"
nav_p.ub_c2 = [0;-3];
nav_p.ub_c3 = [0;-3];
nav_p.ub_c4 = [0;-3];
nav_p.ub_n1 = [0;1]; % Direction of 1-level set
nav_p.ub_n2 = [0;-1];
nav_p.ub_n3 = [1/2;1/2];
nav_p.ub_n4 = [-1/3;-1/4];

theta = 0*pi/180; % [Radian] CounterClockwise | Rotate after
stretch = [1;1];
gamma = 0*pi/180; % [rad] CCW | Post rotate

% Function: g(x):= 1/||x||^alpha
nav_p.alpha = 0.2; % Best value 0.2

% Euler Parameters
N = 5000; %timesteps
deltaT = 0.01;
ctrl_multiplier = 50; % Parameter to change

% Rantzer Check Parameters
check_rantzer = true;
rantzer_graph = true;


% Optimize function handle generation
optimize = false; % Decision to optimize function generation | false for faster generation time
vpa_enable = true;

if vpa_enable
    optimize = false; % Set optimize also false s.t. symbolic tool generation is fast / manageable
end

% Save Figures
save_fig = true;

%% Density Function Formulation
% Create function of density and bump function
syms x [2,1] real

[A, A_inv] = transformationMatrix(theta, stretch, 2);
% Obstacle function

bump = formPolyhedron(repmat(nav_p.r1, 1, 4), repmat(nav_p.r2, 1, 4), ...
        [nav_p.ub_c1, nav_p.ub_c2, nav_p.ub_c3, nav_p.ub_c4], x, ...
        [nav_p.ub_n1, nav_p.ub_n2, nav_p.ub_n3, nav_p.ub_n4], true);

g = 1/norm(x-nav_p.xd)^(2*nav_p.alpha); % rho

% Density function
density = g*bump;
%density = g;

grad_density = gradient(density, x);
hess_density = hessian(density, x);

% Create function handles
% matlabFunction(bump, 'File', 'functions/bump_f', 'Vars', {x}, 'Optimize', optimize);
% matlabFunction(g, 'File', 'functions/rho_f', 'Vars', {x}, 'Optimize', optimize);
matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x}, 'Optimize', optimize);
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x}, 'Optimize', optimize);
%matlabFunction(hess_density, 'File', 'functions/hess_density_f', 'Vars', {x}, 'Optimize', optimize);
toc

%% Plotting Density & Rantzer Conditions
x_vec = -10:0.25:10; y_vec = x_vec;
[X,Y] = meshgrid(x_vec,y_vec);
rho_val = zeros(size(x));
grad_rho_x1 = zeros(size(x));
grad_rho_x2 = zeros(size(x));

for i=1:length(x_vec)
    for j = 1:length(y_vec)
        rho_val(i,j) = density_f([x_vec(j);y_vec(i)]);
        grad_rho = grad_density_f([x_vec(j);y_vec(i)]);
        grad_rho_x1(i,j) = grad_rho(1);
        grad_rho_x2(i,j) = grad_rho(2);
    end
end

dens_fig = surf(X,Y,rho_val, 'FaceAlpha',0.65, 'EdgeColor', 'none');
colormap jet
view(90,60)
%title("Density Function")
set(gca, "FontSize", 15)
xlabel("x_1", "FontSize", 20)
ylabel("x_2", "FontSize", 20)
%xlim([-3 3])
%ylim([-3 3])
grid off
%zlim([0 5])
%axis equal

% grad_fig = figure();
% quiver(X, Y, grad_rho_x1, grad_rho_x2, 0); % Add 0 for no scale
% hold on;
% title("Gradient of Density Function");

log_grad_fig = figure();
quiverInLogScale(X,Y, grad_rho_x1, grad_rho_x2)
title("Log Gradient of Density Function");
xlabel("logx");
ylabel("logy");

% Check Rantzer's Condition Graphically | div (rho u)
% if check_rantzer || rantzer_graph
%     [rantzer_cond, pos_not_rantz_cond, rantzer_fig] = rantzer_check(check_rantzer, rantzer_graph);
% end

%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT);

%% using Euler method
num_pts = 21; % Points on a side

ics = [linspace(-10,10,num_pts), -10*ones(1,num_pts), 10*ones(1,num_pts), linspace(-10, 10, num_pts);
        10*ones(1,num_pts), linspace(-10,10,num_pts), linspace(-10,10,num_pts), -10*ones(1,num_pts)];
% ics = [-10*ones(1,num_pts);
%         linspace(-10,10,num_pts)];
num_ics = size(ics, 2); % Total number of ICs
x_euler_list = zeros(N+1, length(x), num_ics);
u_euler_list = zeros(N, length(x), num_ics);

for i = 1:num_ics
    nav_p.x0 = ics(:,i);
    [x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, ...
                    @singleIntegrator, single_int_p);
    x_euler_list(:,:,i) = x_euler;
    u_euler_list(:,:,i) = u_euler;
end
%% Plot

obsColor = [.7 .7 .7]; % Obstacle color -> Grey


nav_fig = figure(999);
xlim([-5,5]);
ylim([-5,5]);
axis square


plot(nav_p.xd(1), nav_p.xd(2), 'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;

for i=1:num_ics
    state_traj = plot(x_euler_list(1:end,1, i),x_euler_list(1:end,2, i),'b-', 'LineWidth', 2.5); hold on;
end
plot(nav_p.xd(1), nav_p.xd(2), 'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on; % Replot to see the goal

[~, c] = contour(X,Y,rho_val, 10, '-');
c.LineWidth = 1;
lgd = legend('Goal','Trajectory', '');
lgd.FontSize = 14;
set(lgd,'interpreter','latex')

%title('Euclidean Space Trajectory');
set(gca, "FontSize", 15)
xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

figure()
t2 = tiledlayout(2,2, 'Padding', 'Tight');
for i=1:length(x)
nexttile
t = 1:size(x_euler,1);
%state_traj_fig = figure(1000);
plot(t,squeeze(x_euler_list(:,i,:)), 'LineWidth', 2.5); hold on;
%title("State trajectory")
set(gca, "FontSize", 15)
xlabel("Iteration", 'FontSize', 20)
ylabel(sprintf("$x_%d$", i), 'FontSize', 20, 'interpreter', 'latex')
% state_lgd = legend("x1", "x2");
% state_lgd.FontSize = 14;
end

for i=1:length(x)
nexttile
t = 1:size(u_euler,1);
%ctrl_traj_fig = figure(1001);
plot(t,squeeze(u_euler_list(:,i,:)), 'LineWidth', 2.5); hold on;
%title("Control trajectory")
set(gca, "FontSize", 15)
xlabel("Iteration", 'FontSize', 20)
ylabel(sprintf("$u_%d$", i), 'FontSize', 20, 'interpreter', 'latex')
% ctrl_lgd = legend("u1", "u2");
% ctrl_lgd.FontSize = 14;
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