function [robot_params,euler_params,navigation_params,lqr_params,animate_params] = get_params(x)

if(nargin<1)
    x = sym('x',2);
end
%% robotic arm parameters
% robot arm lengths
robot_params.l1 = 1; robot_params.l2 = 1;
robot_params.m1 = 1 ; robot_params.m2 = 1;
robot_params.g = 9.81;
robot_params.I1 = 1 ;robot_params.I2 = 1;

%% robot end effector paramters
% these params are used for normalizing the control input from the planner
% normalize u as max(u)/mu*mg
% has a saturation effect on the controller
% leads to more smooth trajectories
%navigation_params.mu = 0.1; % Friction coefficient [-]
%navigation_params.mg = 20; % Weight [N].
navigation_params.saturation = 1;
navigation_params.rad_from_goal = 0.001;
navigation_params.ctrl_multiplier = 100; % Parameter to change
%navigation_params.ctrl_multiplier = 1;

%% parameters for euler integration
% Target set radius | Radius when to stop using Density FB control
planarKINOVA = 0;
if(planarKINOVA)
    euler_params.n_steps = 500;
    euler_params.step_size = 0.01;
else
    euler_params.n_steps = 10000;
    euler_params.step_size = 0.001;
end

%% animation paramters
animate_params.flag_movie = 1;
animate_params.skip_rate = 50;

% params for dynamic obs case
animate_params.dynamic_obs.traj = 0;
animate_params.dynamic_obs.density_gif = 0;

% params for static obs case
animate_params.static_obs = 1;

% params for plotting obstalce in joint space
animate_params.plot_joint_obs = 1;

%% Form system matrices and LQR Gain
lqr_params = get_LQR(x);
