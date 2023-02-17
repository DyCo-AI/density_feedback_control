function plotfigure2(robot_params,navigation_params,joint_control,joints)
%% get params
a1 = robot_params.l1 ; a2 = robot_params.l2;
grayColor = [.7 .7 .7];
redColor = [1 0 0];
w = 0.05; % width of robotic arm
joint_angles = joint_control.angles;
joint_vel = joint_control.vel;
control = joint_control.control;
time = joint_control.t;

x_goal = navigation_params.x_goal;
n_obs = navigation_params.n_obs;
x_obs = navigation_params.x_obs;
x_obs_rad = navigation_params.x_obs_rad;

load('saved_data/joint_obs')

figure()
%% ------------------------ plot task space solution --------------------------------------------
p1 = subplot(4,4,[1, 5]);
% plot obstacles at t=0;
% plot circles for obstalces
r1 = x_obs_rad(1); r2 = x_obs_rad(2);
angles=linspace(0,2*pi,1000).'; angles(end)=[];
circle1=polyshape([cos(angles), sin(angles)]*r1+[x_obs(1,1),x_obs(1,2)]);
circle2=polyshape([cos(angles), sin(angles)]*r2+[x_obs(2,1),x_obs(2,2)]);
plot(circle1,'FaceColor','black','LineWidth',2); hold on;
plot(circle2,'FaceColor','black','LineWidth',2); hold on;

% get links config
q1 = joints(1,1); q2 = joints(1,2);
x1 = a1*sin(q1); y1 = -a1*cos(q1);
x2 = x1 + a2*sin(q1+q2); y2 = y1 - a2*cos(q1+q2);

% plot initial link position
link1 = polyshape([x1+w*cos(q1) x1-w*cos(q1) -w*cos(q1) +w*cos(q1)], ...
[y1+w*sin(q1) y1-w*sin(q1) -w*sin(q1) w*sin(q1)]);
plot(link1,'FaceColor',redColor, 'FaceAlpha',0.1, 'EdgeColor','black'); hold on
    
link2 = polyshape([x2+w*cos(q1+q2) x2-w*cos(q1+q2) x1-w*cos(q1+q2) x1+w*cos(q1+q2)], ...
[y2+w*sin(q1+q2) y2-w*sin(q1+q2) y1-w*sin(q1+q2) y1+w*sin(q1+q2)]);
plot(link2,'FaceColor',redColor, 'FaceAlpha',0.1, 'EdgeColor','black'); hold on

% plot starting position of end effector
%plot(navigation_params.x_ini(1),navigation_params.x_ini(2),'ob', 'MarkerSize',10, 'MarkerFaceColor','blue'); hold on

% plot goal position on the end effector
plot(navigation_params.x_goal(1),navigation_params.x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on

% plot robot history
idx = [2000,3000,4000,5000,6000,10000];
for angles = 1:length(idx)
    q1 = joints(idx(angles),1); q2 = joints(idx(angles),2);
    x1 = a1*sin(q1); y1 = -a1*cos(q1);
    x2 = x1 + a2*sin(q1+q2); y2 = y1 - a2*cos(q1+q2);
    
    % plot initial link position
    link1 = polyshape([x1+w*cos(q1) x1-w*cos(q1) -w*cos(q1) +w*cos(q1)], ...
    [y1+w*sin(q1) y1-w*sin(q1) -w*sin(q1) w*sin(q1)]);
    plot(link1,'FaceColor',redColor, 'FaceAlpha',0.7^(length(idx)-angles), 'EdgeColor','black'); hold on
        
    link2 = polyshape([x2+w*cos(q1+q2) x2-w*cos(q1+q2) x1-w*cos(q1+q2) x1+w*cos(q1+q2)], ...
    [y2+w*sin(q1+q2) y2-w*sin(q1+q2) y1-w*sin(q1+q2) y1+w*sin(q1+q2)]);
    plot(link2,'FaceColor',redColor, 'FaceAlpha',0.7^(length(idx)-angles), 'EdgeColor','black'); hold on
end

% plot end effector trajectory
% x_ee = FK_planarRR(x_euler - [pi/2 0]);
% plot(x_ee(:,1),x_ee(:,2),'blue', 'LineWidth', 1); hold on;
xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
p1.XLim = [-2.1, 2.1]; p1.YLim = [-2.1, 2.1];
% plot options
axes1 = gca;
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);

%------------ plot joint space solution ----------------------
[X,Y] = meshgrid(-2*pi:0.1:2*pi, -2*pi:0.1:2*pi);
Z = zeros(size(X));
Z_grad_x1 = zeros(size(X));
Z_grad_x2 = zeros(size(X));
for i=1:length(X)
    for j = 1:length(Y)
        Z(i,j) = density_f([X(i,j);Y(i,j)]);
        z_grad = grad_density_f([X(i,j);Y(i,j)]);
        Z_grad_x1(i,j) = z_grad(1);
        Z_grad_x2(i,j) = z_grad(2);
    end
end

% p2 = subplot(4,4,[2,6]);
% scatter(joint_obs(:,1)',joint_obs(:,2)',50,'Marker','square','MarkerEdgeColor',grayColor,...
%           'MarkerFaceColor',grayColor); hold on;
% contour(X,Y,Z); hold on;
% plot(navigation_params.x_ini(1,1),navigation_params.x_ini(1,2), 'ob', 'MarkerSize',10, 'MarkerFaceColor','blue'); hold on;
% plot(x_euler(:,1),x_euler(:,2),'blue', 'LineWidth', 2); hold on;
% plot(navigation_params.x_goal(1),navigation_params.x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
% 
% p2.XLim = [-0.10 2*pi]; p2.YLim = [-0.1 2*pi];
% xticks(0:pi/3:2*pi); yticks(0:pi/3:2*pi);
% xtick = get(gca,'XTick'); ytick = get(gca,'YTick');
% set(gca, 'XTick', xtick,'XTickLabel',xtick.*180/pi)
% set(gca, 'YTick', ytick,'YTickLabel',ytick.*180/pi)
% xlabel('q_1'); ylabel('q_2');

%% ------------------ state traj plots ----------------------------------------------

state_x = joint_angles;
state_xdot = joint_vel;

subplot(4,4,2)
plot(time./1000,state_x(:,1),'LineWidth',2); hold on;
plot(time./1000,state_x(:,2),'LineWidth',2); hold on;
plot(time./1000,state_xdot(:,1),'LineWidth',2); hold on;
plot(time./1000,state_xdot(:,2),'LineWidth',2); hold on;
xlabel('time (s)','interpreter','latex', 'FontSize', 20);
ylabel('states','interpreter','latex', 'FontSize', 20);
axes1 = gca;
box(axes1,'on');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);

subplot(4,4,6)
plot(time./1000,control(:,1),'LineWidth',2); hold on;
plot(time./1000,control(:,2),'LineWidth',2);
xlabel('time (s)','interpreter','latex', 'FontSize', 20);
ylabel('joint angles','interpreter','latex', 'FontSize', 20);
axes1 = gca;
box(axes1,'on');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);