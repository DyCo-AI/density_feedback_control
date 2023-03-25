%% Obstacle Avoidance using single integrator dynamics
clc
clear
close all
%% Problem setup
mkdir('functions');
mkdir('utils')
mkdir('dynamics-control')
mkdir('bump_lib')
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('bump_lib')

tic
% Start position
nav_p.x0 = [0;-10;-8]; % Original: [1;9;9]
% nav_p.xd = [4;4;4];
nav_p.xd = [4;4;8];

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 0.5;
nav_p.alpha = 0.2;

N = 20000; %timesteps
deltaT = 0.01;
ctrl_mult = 100; % Parameter to change

colors = colororder;
blue = colors(1,:);
red = colors(2,:);
yellow = colors(3,:);
green = colors(5,:);
obsColor = [.7 .7 .7]; % Obstacle color -> Grey

%% Form Torus good
syms x [3 1] real
c = [0;0]; p = 2;

bump  = 1;

% torus 1 in XY
cx = 0; cy = 0; cz = 0; % center
R = 10; % distance from center
r = 2; % radius of torus
r1 = r; r2 = r1+1; % for sensing
bump = formTorus([cx, cy, cz], R, r, r2, x);

%torus 2 in XZ
cx = 12; cy = 0; cz = 0; % center
R = 7; % distance from center
r = 2; % radius of torus
r1 = r; r2 = r1+1; % for sensing
shape = (sqrt((x(1)-cx)^2 + (x(3)-cz)^2) -R)^2 + (x(2)-cy)^2 - r^2;
shift = 1; % to get rid of random zeros
h =  shape + shift;
m = h/(r2^2-r1^2);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
% Normalize the symmetric step function
bump = bump*(f/(f+f_shift));

%cylinder
cx = 0; cy = 0; cz = 0; % center
r = 2; % radius of cylinder
r1 = r; r2 = r+1; % for sensing
bump = bump*formCylinder([cx,cy,cz], r1, r2, x);

%shpere
c = [0;-6;-4]; % center
r = 3; % radius of shpereeeee
r1 = r; r2 = r+1; % for sensing
shape = norm(x-c)^2 - r^2;
shift = 1; % to get rid of random zeros
h =  shape + shift;
m = h/(r2^2-r1^2);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
% Normalize the symmetric step function
bump = bump*(f/(f+f_shift));

piecewise_f = bump;

%goal

g = 1/norm(x-nav_p.xd)^(2*nav_p.alpha); %

% Density function
density = g*piecewise_f;

bump_grad = gradient(bump,x);
grad_density = gradient(density, x);


matlabFunction(bump, 'File', 'functions/bump_f', 'Vars', {x});
matlabFunction(bump_grad, 'File', 'functions/bump_grad_f', 'Vars', {x});
matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x});
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x});

%% 3D
[XX,YY,ZZ] = meshgrid(-25:0.25:25, -25:0.25:25, -25:0.25:25);
bump_val = zeros(size(XX));
bump_grad_x1 = zeros(size(XX));
grad_rho_x2 = zeros(size(XX));
for i=1:length(XX)
    for j = 1:length(YY)
        for k = 1:length(ZZ)
            bump_val(i,j,k) = bump_f([XX(i,j,k);YY(i,j,k);ZZ(i,j,k)]);
        end
    end
end


%% using Euler method
[single_int_p, dbl_int_p] = generateStateSpace(deltaT, x);
[x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_mult, @singleIntegrator, single_int_p);
toc

%% 3d plot
% figure()
% plot torus 1
subplot(1,4,2)
cx = 0; cy = 0; cz = 0; % center
[theta,phi] = meshgrid(linspace(0,2*pi,25));
R = 10; r = 2;
x = (R + r*cos(theta)).*cos(phi) + cx;
y = (R + r*cos(theta)).*sin(phi) + cy;
z = r*sin(theta) + cz;
surf(x,y,z); hold on
axis equal

% plot torus 2
R = 7; r = 2;
cx = 12; cy = 0; cz = 0; % center
x = (R + r*cos(theta)).*cos(phi) + cx;
z = (R + r*cos(theta)).*sin(phi) + cz;
y = r*sin(theta) + cy;
surf(x,y,z); hold on
axis equal

%  plot cylinder
r = 2; h = 40;
[X,Y,Z] = cylinder(r,20);
Z = Z*h;
surf(X,Y,Z-20); hold on
axis equal

% plot shere
[X,Y,Z] = sphere;
r = 3;
X = X * r;
Y = Y * r;
Z = Z * r;
surf(X+c(1),Y+c(2),Z+c(3)); hold on
axis equal
colormap gray

% bump_val2 = bump_val;
% bump_val2(bump_val2~=0) = nan;
% scatter3(XX(:),YY(:),ZZ(:),40,bump_val2(:),'filled'); hold on

% plot start, goal and traj
plot3(nav_p.x0(1), nav_p.x0(2), nav_p.x0(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'black', 'MarkerEdgeColor','black'); hold on;
plot3(nav_p.xd(1), nav_p.xd(2), nav_p.xd(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', green, 'MarkerEdgeColor', green); hold on;
plot3(x_euler(:,1), x_euler(:,2), x_euler(:,3), 'color', red, 'LineWidth', 2)

%zlim([-0.5 2])
view([45,25]);
axes1 = gca;
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
grid on;
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5)

xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
zlabel('$x_3$','interpreter','latex', 'FontSize', 20);

% t = 1:size(x_euler,1);
% figure()
% plot(t,x_euler)
% xlabel("Iteration")
% ylabel("States")
% legend("x1", "x2", "x3")
% 
% t = 1:size(u_euler,1);
% figure()
% plot(t,u_euler)
% xlabel("Iteration")
% ylabel("Control")
% legend("u1", "u2", "u3")