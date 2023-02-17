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
addpath('./bump_lib');

tic

% Start position
nav_p.x0 = [-2;-2.5]; % Start position which does not converge
nav_p.xd = [3;0]; % Final goal position

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 0.1;

% Euler Parameters
N = 5000; %timesteps
deltaT = 0.01;

% density params
ctrl_multiplier = 500; % Parameter to change
nav_p.alpha = 0.1; % Best value 0.2

% NF params
nav_p.r0 = 1000; % Radius of environment
nav_p.k = 10; % Tuning parameter
ctrl_multiplier_nf = 1e15; % Parameter to change

colors = colororder;
blue = colors(1,:);
red = colors(2,:);
yellow = colors(3,:);
green = colors(5,:);
obsColor = [.7 .7 .7]; % Obstacle color -> Grey

%% Density Function Formulation
syms x [2 1]
% for square bump
% theta = 45*pi/180; p = 1; 
theta = 0*pi/180; p = 2; 
bump = 1;

% square 1 with elliptical sensing
c = [-0.75;0]; a = 0.1; b = 0.05;
r1 = 1; r2 = 45; %best r2 = 45
circ = (x(1)-c(1))^2 + (x(2)-c(2))^2 - 1;
ellipse = (x(1)-c(1))^2/a^2 + (x(2)-c(2))^2/b^2 - r1^2;
egg = (x(1)-c(1))^2/a^2 + 1.4^(x(1))*(x(2)-c(2))^2/b^2 - r1^2;
m = egg/(r2^2-r1^2);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
g1 = f/(f+f_shift);

r1 = 1; r2 = 1.2; c = [0;0];
% r1 = r1/sqrt(2); r2 = r2/sqrt(2); % for one norm unit square
stretch = inv(diag([1 2])); 
Rotate = [cos(theta) -sin(theta); sin(theta) cos(theta)];
A = Rotate*stretch; %stretch and rotate.
m = (norm(A*(x-c),p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
g2 = f/(f+f_shift);
bump = bump*g1*g2; % Emphasis that g is bump function

% square 2
r1 = 1; r2 = 1.1; c = [-2;-2];
% r1 = r1/sqrt(2); r2 = r2/sqrt(2); % for one norm unit square
stretch = inv(diag([2 1])); 
Rotate = [cos(theta) -sin(theta); sin(theta) cos(theta)];
A = Rotate*stretch; %stretch and rotate.
m = (norm(A*(x-c),p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
g2 = f/(f+f_shift);
bump = bump*g2;

% square 3
r1 = 1; r2 = 1.1; c = [-2;2];
% r1 = r1/sqrt(2); r2 = r2/sqrt(2); % for one norm unit square
stretch = inv(diag([2 1])); 
Rotate = [cos(theta) -sin(theta); sin(theta) cos(theta)];
A = Rotate*stretch; %stretch and rotate.
m = (norm(A*(x-c),p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
g2 = f/(f+f_shift);
bump = bump*g2;

piecewise_f = bump;
% Function: g(x):= 1/||x||^alpha
g = 1/norm(x-nav_p.xd)^(2*nav_p.alpha); %

% Density function
density = piecewise_f*g;
grad_density = gradient(density, x);

% Create function handles
matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x},'Optimize',true);
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x},'Optimize',true);
toc
%% Plotting Density Function & Rantzer's Condition
x = -10:0.025:10;
y = x;
[X,Y] = meshgrid(x,y);
rho_val = zeros(size(x));
grad_rho_x1 = zeros(size(x));
grad_rho_x2 = zeros(size(x));

for i=1:length(x)
    for j = 1:length(y)
        Z(i,j) = density_f([x(j);y(i)]);
        z_grad = grad_density_f([x(j);y(i)]);
        Z_grad_x1(i,j) = z_grad(1);
        Z_grad_x2(i,j) = z_grad(2);
    end
end
figure(3)
axes1 = gca;
surf(X,Y,Z, 'FaceAlpha',1, 'EdgeColor', 'flat'); hold on;
contour(X,Y,Z,'LineWidth',2,'LevelStep',0.2,...
    'ZLocation','zmin');
hold on;
colormap jet
grid(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15);
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
view([45,25]);
zlim([-1,3])

%% for NF
syms x [2 1] real
% Environmental Parameter
% % L shape success
r11 = 1;    r12 = 5;         c1 = [-1; -2];
r21 = 1;    r22 = 1.5;       c2 = [-2; -2];
r31 = 1;    r32 = 1.5;       c3 = [-3; -2];
r41 = 1;    r42 = 5;         c4 = [0;-1];
r51 = 1;    r52 = 1.5;       c5 = [0;0];
r61 = 1;    r62 = 1.5;       c6 = [0;1];
r71 = 1;    r72 = 1.5;       c7 = [-1;2];
r81 = 1;    r82 = 1.5;       c8 = [-2;2];
r91 = 1;    r92 = 1.5;       c9 = [-3;2];
nav_p.c0 = [0;0]; % Center

% Calculate intermediate variables
gamma_d = norm(x-nav_p.xd)^2;
beta_0 = -norm(x-nav_p.c0)^2 + nav_p.r0^2;

beta_1 = norm(x-c1)^2 - r11^2;
beta_2 = norm(x-c2)^2 - r21^2;
beta_3 = norm(x-c3)^2 - r31^2;
beta_4 = norm(x-c4)^2 - r41^2;
beta_5 = norm(x-c5)^2 - r51^2;
beta_6 = norm(x-c6)^2 - r61^2;
beta_7 = norm(x-c7)^2 - r71^2;
beta_8 = norm(x-c8)^2 - r81^2;
beta_9 = norm(x-c9)^2 - r91^2;

% Optimization parameters
optimize_bool = true;

beta = beta_0*beta_1*beta_2*beta_3*beta_4*beta_5*beta_6*beta_7*beta_8*beta_9;
phi = gamma_d/((gamma_d^nav_p.k + beta)^(1/nav_p.k));
grad_phi = gradient(phi);

matlabFunction(phi, 'File', 'functions/phi_f', 'Vars', {x});
matlabFunction(grad_phi, 'File', 'functions/grad_phi_f', 'Vars', {x}, 'Optimize', optimize_bool);

%% Plotting Density Function & Rantzer's Condition
x = -10:0.025:10;
y = x;
[X,Y] = meshgrid(x,y);
NF_val = zeros(size(x));
grad_NF_x1 = zeros(size(x));
grad_NF_x2 = zeros(size(x));

for i=1:length(x)
    for j = 1:length(y)
        Z_NF(i,j) = phi_f([x(j);y(i)]);
        z_grad_NF = grad_phi_f([x(j);y(i)]);
        Z_grad_x1_NF(i,j) = z_grad_NF(1);
        Z_grad_x2_NF(i,j) = z_grad_NF(2);
    end
end
figure(4)
axes1 = gca;
surf(X,Y,real(Z_NF), 'FaceAlpha',1, 'EdgeColor', 'flat'); hold on;
contour(X,Y,real(Z_NF),'LineWidth',2,'LevelStep',0.2,...
    'ZLocation','zmin');
hold on;
colormap jet
grid(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15);
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
view([45,25]);
zlim([-1,3])
%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT);

%% using Euler method

% initial conditions around a box
num_pts = 100;
lx = [-4;-2]; ly = [-0.5;0.5];
x = rand(1,num_pts)*range(lx) + lx(1);
y = rand(1,num_pts)*range(ly) + ly(1);
ics = [x;y];
num_ics = size(ics, 2); % Total number of ICs

% get density states
x_euler_list = zeros(N+1, length(nav_p.xd), num_ics);
u_euler_list = zeros(N, length(nav_p.xd), num_ics);
for i = 1:num_ics
    nav_p.x0 = ics(:,i);
    [x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, ...
                    @singleIntegrator, single_int_p);
    x_euler_list(:,:,i) = x_euler;
    u_euler_list(:,:,i) = u_euler;
end

% get NF states
x_euler_list_nf = zeros(N+1, length(nav_p.xd), num_ics);
u_euler_list_nf = zeros(N, length(nav_p.xd), num_ics);
for i = 1:num_ics
  nav_p.x0 = ics(:,i);
  [x_euler_nf, u_euler_nf] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier_nf, ...
                    @singleIntegrator_potential, single_int_p);
    x_euler_list_nf(:,:,i) = x_euler_nf;
    u_euler_list_nf(:,:,i) = u_euler_nf;
end

% test traj
% [x_euler_nf, u_euler_nf] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier_nf, @singleIntegrator_potential, single_int_p);

%% Plot density traj
% figure(1)
%subplot(4,2,[1,3,5,7])

contour(X,Y,Z,'LineWidth',2,'LevelStep',0.2); hold on;
gamma = (0:100-1)*(2*pi/100);
points = [0;0] + [1*cos(gamma);2*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = [-2;2] + [2*cos(gamma);1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = [-2;-2] + [2*cos(gamma);1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;


% plot traj from initial conditions around a box
for i=1:num_ics
    plot(x_euler_list(:,1,i),x_euler_list(:,2,i),'color',red, 'LineWidth', 2); hold on;
end

for i=1:num_ics
    plot(x_euler_list(1,1,i),x_euler_list(1,2,i),'o', 'MarkerSize',2, 'MarkerFaceColor','black','MarkerEdgeColor','black'); hold on;
end

%plot goal
plot(nav_p.xd(1), nav_p.xd(2), 'o', 'MarkerSize',10, 'MarkerFaceColor',green,'MarkerEdgeColor',green); hold on;

%plot dummy marker for obs
dummy_marker = plot(NaN,NaN, 'o','MarkerSize', 10, 'MarkerEdgeColor',...
            'black', 'MarkerFaceColor',obsColor, 'LineWidth', 1.5); hold on;

% plot options
axes1 = gca;
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);

ylim([-7,7]);
xlim([-7,7]);
axis 'square'

xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

%% state traj list for density
subplot(4,2,2)
t = 1:size(x_euler_list,1);
plot(t.*deltaT,squeeze(x_euler_list(:,1,:)), 'LineWidth', 2); hold on;
axes1 = gca;
box(axes1,'on');
axis(axes1,'tight');
hold(axes1,'off');
set(axes1,'FontSize',15,'LineWidth',1.5,...
   'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$x_1$','interpreter','latex', 'FontSize', 20);

subplot(4,2,4)
t = 1:size(x_euler_list,1);
plot(t.*deltaT,squeeze(x_euler_list(:,2,:)), 'LineWidth', 2); hold on;
axes2 = gca;
box(axes2,'on');
axis(axes2,'tight');
hold(axes2,'off');
set(axes2,'FontSize',15,'LineWidth',1.5,...
    'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

subplot(4,2,6)
t = 1:size(u_euler_list,1);
plot(t.*deltaT,squeeze(u_euler_list(:,2,:)), 'LineWidth', 2); hold on;
axes2 = gca;
box(axes2,'on');
axis(axes2,'tight');
hold(axes2,'off');
set(axes2,'FontSize',15,'LineWidth',1.5,...
    'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$u_1$','interpreter','latex', 'FontSize', 20);

subplot(4,2,8)
t = 1:size(u_euler_list,1);
plot(t.*deltaT,squeeze(u_euler_list(:,2,:)), 'LineWidth', 2); hold on;
axes2 = gca;
box(axes2,'on');
axis(axes2,'tight');
hold(axes2,'off');
set(axes2,'FontSize',15,'LineWidth',1.5,...
    'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$u_2$','interpreter','latex', 'FontSize', 20);

%% Plot NF traj
%figure(2)
%subplot(4,2,[1,3,5,7])

%plot obs
contour(X,Y,real(Z_NF),'LineWidth',2,'LevelStep',0.2); hold on;

gamma = (0:100-1)*(2*pi/100);
points = c1 + [r11*cos(gamma);r11*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c2 + [r21*cos(gamma);r21*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c3 + [r31*cos(gamma);r31*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c4 + [r41*cos(gamma);r41*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c5 + [r51*cos(gamma);r51*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c6 + [r61*cos(gamma);r61*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c7 + [r71*cos(gamma);r71*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c8 + [r81*cos(gamma);r81*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

points = c9 + [r91*cos(gamma);r91*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;


% plot traj from initial conditions around a box
for i=1:num_ics
    plot(x_euler_list_nf(:,1,i),x_euler_list_nf(:,2,i),'color',red, 'LineWidth', 2); hold on;
end

for i=1:num_ics
    plot(x_euler_list(1,1,i),x_euler_list(1,2,i),'o', 'MarkerSize',2, 'MarkerFaceColor','black','MarkerEdgeColor','black'); hold on;
end

%plot goal
plot(nav_p.xd(1), nav_p.xd(2), 'o', 'MarkerSize',10, 'MarkerFaceColor',green,'MarkerEdgeColor',green); hold on;

%plot dummy marker for obs
dummy_marker = plot(NaN,NaN, 'o','MarkerSize', 10, 'MarkerEdgeColor',...
            'black', 'MarkerFaceColor',obsColor, 'LineWidth', 1.5); hold on;

% plot options
axes1 = gca;
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);

ylim([-7,7]);
xlim([-7,7]);
axis 'square'

xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

%% state traj list for NF
subplot(4,2,2)
t = 1:size(x_euler_list_nf,1);
plot(t.*deltaT,squeeze(x_euler_list_nf(:,1,:)), 'LineWidth', 2); hold on;
axes1 = gca;
box(axes1,'on');
axis(axes1,'tight');
hold(axes1,'off');
set(axes1,'FontSize',15,'LineWidth',1.5,...
   'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$x_1$','interpreter','latex', 'FontSize', 20);

subplot(4,2,4)
t = 1:size(x_euler_list_nf,1);
plot(t.*deltaT,squeeze(x_euler_list_nf(:,2,:)), 'LineWidth', 2); hold on;
axes2 = gca;
box(axes2,'on');
axis(axes2,'tight');
hold(axes2,'off');
set(axes2,'FontSize',15,'LineWidth',1.5,...
    'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

subplot(4,2,6)
t = 1:size(u_euler_list_nf,1);
plot(t.*deltaT,squeeze(u_euler_list_nf(:,2,:)), 'LineWidth', 2); hold on;
axes2 = gca;
box(axes2,'on');
axis(axes2,'tight');
hold(axes2,'off');
set(axes2,'FontSize',15,'LineWidth',1.5,...
    'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$u_1$','interpreter','latex', 'FontSize', 20);

subplot(4,2,8)
t = 1:size(u_euler_list_nf,1);
plot(t.*deltaT,squeeze(u_euler_list_nf(:,2,:)), 'LineWidth', 2); hold on;
axes2 = gca;
box(axes2,'on');
axis(axes2,'tight');
hold(axes2,'off');
set(axes2,'FontSize',15,'LineWidth',1.5,...
    'XLimitMethod','tight',...
    'YLimitMethod','tight','ZLimitMethod','tight');
xlabel('$time$','interpreter','latex', 'FontSize', 20);
ylabel('$u_2$','interpreter','latex', 'FontSize', 20);