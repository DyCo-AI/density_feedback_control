%% Obstacle Avoidance using single integrator dynamics
clc
clear
close all
%% Problem setup
mkdir('functions');
mkdir('utils')
mkdir('dynamics-control')
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('bump_lib');


% TODO: Generate graphically the obstacle s.t. p-norm shaped
tic
% Start position
nav_p.x0 = [0;10;5]; % Original: [1;9;9]
nav_p.xd = [0;-1;5];

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 0.5;

% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
nav_p.a = 0.1; % Positive definite | Best value 0.1
nav_p.r1 = 1.4; % Change radius: increase r -> increase radius
nav_p.r2 = 1.5; % TODO: Try r2 = 1.8
nav_p.r1 = [0.75, 0.75, 0.75, 0.75, 1, 1, 1, 1, 1, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25];
nav_p.r2 = nav_p.r1 + 0.25;
nav_p.p = 2; % p-norm obstacle shape

% Obstancle centers (e.g. c1 and c2) | Domain R^n
num_obs = 15; % Number of obstacles

nav_p.c1 = [3; 8; 8]; % Vertices of box
nav_p.c2 = [-3; 8; 8];
nav_p.c3 = [3; 8; 2];
nav_p.c4 = [-3; 8; 2];
nav_p.c5 = [3; 2;8];
nav_p.c6 = [-3; 2; 8];
nav_p.c7 = [3; 2; 2];
nav_p.c8 = [-3; 2; 2];
nav_p.c9 = [0; 5; 5]; % Center
nav_p.c10 = [-3; 5; 5]; % Left center
nav_p.c11 = [3; 5; 5]; % Right Center
nav_p.c12 = [0; 8; 5.1]; % Back Center
nav_p.c13 = [0; 2; 5]; % Front Center
nav_p.c14 = [0; 5; 8]; % Top Center
nav_p.c15 = [0; 5; 2]; % Bottom Center



% Function: g(x):= 1/||x||^alpha
nav_p.alpha = 0.2; % Best value 0.2

% Forward Euler Parameters
N = 15000; %timesteps
deltaT = 0.01;
ctrl_mult = 20;

% Plot misc graph
plot_misc_graph = false;
plot_mult_ic = true;

% Optimize function handle generation
optimize = false; % Decision to optimize function generation | false for faster generation time
vpa_enable = true;

if vpa_enable
    optimize = false; % Set optimize also false s.t. symbolic tool generation is fast / manageable
end

%% Density Function Formulation

% Create function of density and bump function
syms x [3,1] real % Change if going higher dim

% Obstacle function TODO: Make into function and update meshgrid
bump = formFastInvBump(x, nav_p, num_obs, eye(3));

% Density function
g = 1/norm(x-nav_p.xd)^(2*nav_p.alpha); % rho


density = g*bump;

grad_density = gradient(density, x);
hess_density = hessian(density, x);

% Create function handles
%matlabFunction(piecewise_f, 'File', 'functions/bump_f', 'Vars', {x}, 'Optimize', optimize);
%matlabFunction(g, 'File', 'functions/rho_f', 'Vars', {x}, 'Optimize', optimize);
matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x}, 'Optimize', optimize);
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x}, 'Optimize', optimize);
%matlabFunction(hess_density, 'File', 'functions/hess_density_f', 'Vars', {x}, 'Optimize', optimize);


toc
%% Plotting Density Function
if plot_misc_graph

    x_vec = -10:0.25:10; y_vec = x_vec; z_vec = 0:0.25:10;
    [X,Y,Z] = meshgrid(x_vec, y_vec, z_vec);
    dens = zeros(size(X));
    grad_dens_x1 = zeros(size(X));
    grad_dens_x2 = zeros(size(X));
    grad_dens_x3 = zeros(size(X));
    for i=1:length(X)
        for j = 1:length(Y)
            for k = 1:length(Z)
                dens(i,j,k) = density_f([X(i,j);Y(i,j); Z(i,j)]);
                grad_dens = grad_density_f([X(i,j);Y(i,j); Z(i,j)]);
                grad_dens_x1(i,j,k) = grad_dens(1);
                grad_dens_x2(i,j,k) = grad_dens(2);
                grad_dens_x3(i,j,k) = grad_dens(3);
            end
        end
    end

    scalingFactor = 2; % Helper variable to see gradients more clearly
    %figure()
    %quiver3(X, Y, Z, scalingFactor*grad_dens_x1, scalingFactor*grad_dens_x2, scalingFactor*grad_dens_x3, 0);
    %title("Gradient of Density Function")

end
%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT, x);

%% using Euler method

if plot_mult_ic
    num_pts = 5; % Points on a side
    ics = [-4*ones(1,num_pts), -2*ones(1,num_pts), 0*ones(1,num_pts), 2*ones(1,num_pts), 4*ones(1,num_pts);
        10*ones(1,num_pts), 10*ones(1,num_pts), 10*ones(1,num_pts), 10*ones(1,num_pts), 10*ones(1,num_pts);
        linspace(0,10,num_pts), linspace(0,10,num_pts), linspace(0,10,num_pts), linspace(0,10,num_pts), linspace(0,10,num_pts)];
    num_ics = size(ics, 2); % Total number of ICs
    
    % Plot
    figure()
    [x_pts,y_pts,z_pts] = sphere(20);
    % b(x_pts-c)^2 = x_new^2 | x_pts input, x_new is output
    % To graph new x's:
    % x_new = x_pt/sqrt(b)+c
    
    for i=1:num_obs
        if length(nav_p.r1) == 1
            s = surf(x_pts*nav_p.r1+ nav_p.(sprintf("c%d", i))(1), y_pts*nav_p.r1 + nav_p.(sprintf("c%d", i))(2), ...
                z_pts*nav_p.r1 + nav_p.(sprintf("c%d", i))(3));
            hold on;
            s.EdgeColor = 'none';
            s.FaceAlpha = 0.6;
        else
            s = surf(x_pts*nav_p.r1(i)+ nav_p.(sprintf("c%d", i))(1), y_pts*nav_p.r1(i) + nav_p.(sprintf("c%d", i))(2), ...
                z_pts*nav_p.r1(i) + nav_p.(sprintf("c%d", i))(3));
            hold on;
            s.EdgeColor = 'none';
            s.FaceAlpha = 0.6;
        end
    end
    
    for i = 1:num_ics
        nav_p.x0 = ics(:,i);
        [x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_mult, @singleIntegrator, single_int_p);
        
        toc
        
        
        
        axis equal
        plot3(nav_p.xd(1), nav_p.xd(2), nav_p.xd(3), 'og', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); hold on;
        plot3(x_euler(:,1), x_euler(:,2), x_euler(:,3), 'blue', 'LineWidth', 3)
        xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
        ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
        zlabel('$x_3$','interpreter','latex', 'FontSize', 20);
        xlim([-5 5])
        ylim([-1 10])
        zlim([0 10])
        
    end
else
    figure()
    [x_pts,y_pts,z_pts] = sphere(20);
    for i=1:num_obs
        if length(nav_p.r1) == 1
            s = surf(x_pts*nav_p.r1+ nav_p.(sprintf("c%d", i))(1), y_pts*nav_p.r1 + nav_p.(sprintf("c%d", i))(2), ...
                z_pts*nav_p.r1 + nav_p.(sprintf("c%d", i))(3));
            hold on;
            s.EdgeColor = 'none';
            s.FaceAlpha = 0.6;
        else
            s = surf(x_pts*nav_p.r1(i)+ nav_p.(sprintf("c%d", i))(1), y_pts*nav_p.r1(i) + nav_p.(sprintf("c%d", i))(2), ...
                z_pts*nav_p.r1(i) + nav_p.(sprintf("c%d", i))(3));
            hold on;
            s.EdgeColor = 'none';
            s.FaceAlpha = 0.6;
        end
    end
    
    [x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_mult, @singleIntegrator, single_int_p);
    axis equal
    plot3(nav_p.xd(1), nav_p.xd(2), nav_p.xd(3), 'og', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); hold on;
    plot3(x_euler(:,1), x_euler(:,2), x_euler(:,3), 'blue', 'LineWidth', 3)
    xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
    ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
    zlabel('$x_3$','interpreter','latex', 'FontSize', 20);
    xlim([-5 5])
    ylim([-1 10])
    zlim([0 10])
end

t = 1:size(x_euler,1);
figure()
plot(t,x_euler)
xlabel("Iteration")
ylabel("States")
legend("x1", "x2", "x3")


t = 1:size(u_euler,1);
figure()
plot(t,u_euler)
xlabel("Iteration")
ylabel("Control")
legend("u1", "u2", "u3")




%% helper functions
% Currently not being used (used for ode45)
function [value1, value2, isterminal, direction] = detect_NaN(T, Y)
% collect y and t values when NaN occurs during ODE45
value1      = (isnan(Y(1)));
value2      = (isnan(Y(2)));
isterminal = 1;   % Stop the integration
direction  = 0;
end