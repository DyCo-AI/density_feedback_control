%% Obstacle Avoidance for planar robotic arm
clc; clear; close all
addpath(genpath('./navigation'))
addpath(genpath('robotic_arm'))
addpath(genpath('saved_data'))

% Delete the funcs before running
delete('navigation\func\*')
% else run twice for changes in density to update

%% working obstacles and configurations
% 1 random
%navigation_params.x_obs = [-0.7, 0.5];
%navigation_params.x_ini = IK_planarRR([-0.5 -0.6]);
%navigation_params.x_goal = IK_planarRR([-0.5 0.8]);

% 2 random
%navigation_params.x_obs = [1.2, -0.5];
%navigation_params.x_ini = IK_planarRR([-1 -1]);
%navigation_params.x_goal =[0.95 2.4];

% 3 swing up**
%navigation_params.x_obs = [1.2, -0.5];
%navigation_params.x_ini = IK_planarRR([0 -2]);
%navigation_params.x_goal =IK_planarRR([0 2]);

% 4 start close to obstacle
%navigation_params.x_obs = [1.2, -0.5; ...
%                          -1.2, -0.5];
% navigation_params.x_ini = IK_planarRR([0.5 -0.1]);
% navigation_params.x_goal =IK_planarRR([0 2]);

%% Problem setup
x = sym('x',[2 1],'real'); %joint states
[robot_params,euler_params,navigation_params,lqr_params,animate_params] = get_params(x);

% define start and goal in joint space
%navigation_params.x_ini = [0.001 0.001];
navigation_params.x_ini = [0.01 0.01];
navigation_params.x_goal =IK_planarRR([0 2]);

% define obstacles
navigation_params.dynamic_obs = 0;
navigation_params.n_obs = 2;
% navigation_params.x_obs = [1.2, -0.5; ...
%                           -1.2, -0.5];
navigation_params.x_obs = [1.5, -0.5; ...
                          -1.5, -0.5];
navigation_params.x_obs_rad = 0.2*ones(navigation_params.n_obs);

params.flag_movie = 1;
params.dyanmic_traj = 1;
params.density_gif = 0;
params.static_obs = 0;

%% get obs in joint space (uncomment to generate obs for new configuration)
%[joint_obs, convex_hull] = gen_joint_obs(navigation_params);
%save('saved_data/joint_obs','joint_obs')

%% get density function
load('saved_data/joint_obs','joint_obs')
navigation_funs = get_density_joint2(navigation_params,joint_obs);

%% Create function handles
obs_funs = [];
matlabFunction(navigation_funs.piecewise_f, 'File', './navigation/func/bump_f', 'Vars', {x}, 'Optimize', false);
matlabFunction(navigation_funs.V, 'File', './navigation/func/rho_f', 'Vars', {x}, 'Optimize', false);
matlabFunction(navigation_funs.density, 'File', './navigation/func/density_f', 'Vars', {x}, 'Optimize', false);
matlabFunction(navigation_funs.grad_density, 'File', './navigation/func/grad_density_f', 'Vars', {x}, 'Optimize', false);
%matlabFunction(navigation_funs.hess_density, 'File', './navigation/func/hess_density_f', 'Vars', {x}, 'Optimize', false);
rehash path

%% using Euler method
[x_euler, u_euler] = forwardEuler(@singleIntegrator,navigation_params);
x_euler = x_euler';

%% plot density
plot_density_joint(x_euler, navigation_params, obs_funs)

%% plot state & control trajectory
% figure()
% plot(x_euler)
% legend("x1", "x2");
% xlabel("Iteration");
% ylabel("States");
% 
% figure()
% plot(u_euler');
% legend("u1", "u2");
% xlabel("Iterations");
% ylabel("Control");

%% animate plan
joint_plan.x = x_euler;
joint_plan.t = euler_params.step_size:euler_params.n_steps;
time = joint_plan.t; % to do - run LQR before animate

joints = joint_plan.x;
ee_plan = [];
x_ini = FK_planarRR(navigation_params.x_ini - [pi/2 0]);
x_goal = FK_planarRR(navigation_params.x_goal - [pi/2 0]);
animate_planarRR(time,joints,navigation_params,obs_funs,ee_plan,x_ini,x_goal)

%% get inverse dynamics 
q_des = x_euler; q_dot_des = u_euler; q_ddot_des = [diff(u_euler(1,:));diff(u_euler(2,:))]; 
joint_control = ID_planarRR(q_des,q_dot_des,q_ddot_des,robot_params,navigation_params,euler_params);

%% plot
plot_density_joint(joint_control.angles, navigation_params, obs_funs)

%% animate control
joints = joint_control.angles;
joint_control.t = euler_params.step_size:euler_params.n_steps;
animate_planarRR(time,joints,navigation_params,obs_funs,ee_plan,x_ini,x_goal)

%% save plan and paraters
% save('saved_data/navigation_params','navigation_params')

%% plots
%plot_density_joint(x_euler, navigation_params, obs_funs)
plotfigure2(robot_params,navigation_params,joint_control,joints);
