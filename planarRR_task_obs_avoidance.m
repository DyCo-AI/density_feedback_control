%% Obstacle Avoidance for planar robotic arm
clc; clear; close all
addpath(genpath('./navigation'))
addpath(genpath('robotic_arm'))
% Delete the funcs before running

%% Problem setup
x = sym('x',[2 1],'real');
[robot_params,euler_params,navigation_params,lqr_params,animate_params] = get_params(x);

% define start and goal
navigation_params.x_ini = [0.1;0.1];
navigation_params.x_goal = [1;1.5];

% define obstacles
navigation_params.dynamic_obs = 0;
if(navigation_params.dynamic_obs)
    t = sym('t');
    % Obstancle centers (e.g. c1 and c2) | Domain R^n
    %params.obstacle_freq = 1/(4*pi); % [Hz]
    navigation_params.obstacle_freq = 1.2; % [vel]
    
    navigation_params.x_obs = [navigation_params.obstacle_freq*t-2.5, 1.5;
                   1.5 - navigation_params.obstacle_freq*t, -navigation_params.obstacle_freq*t+2.5];

%     params.x_obs = [params.obstacle_freq*t-2, 1];
    navigation_params.n_obs = size(params.x_obs,1);
    navigation_params.x_obs_rad = 0.3*ones(navigation_params.n_obs);
    
else
    navigation_params.n_obs = 1;
    navigation_params.x_obs = [0.42, -0.5];
    navigation_params.x_obs_rad = 0.2*ones(navigation_params.n_obs);
end

params.flag_movie = 1;
params.dyanmic_traj = 1;
params.density_gif = 0;
params.static_obs = 0;

%% get density function
navigation_funs = get_density(navigation_params);

% Create function handles

if(navigation_params.dynamic_obs)
    obs_funs = cell(1,params.n_obs);
    for i=1:params.n_obs
        obs_fun_name = strcat('./navigation/func/obs_fun',num2str(i));
        obs_funs{i} = ...
        matlabFunction(params.x_obs(i,:), 'File', obs_fun_name, 'Vars', {t});
    end
    
    matlabFunction(navigation_funs.piecewise_f, 'File', './navigation/func/bump_f', 'Vars', {x,t});
    matlabFunction(navigation_funs.g, 'File', './navigation/func/rho_f', 'Vars', {x,t});
    matlabFunction(navigation_funs.density, 'File', './navigation/func/density_f', 'Vars', {x,t});
    matlabFunction(navigation_funs.grad_density, 'File', './navigation/func/grad_density_f', 'Vars', {x,t});
    matlabFunction(navigation_funs.hess_density, 'File', './navigation/func/hess_density_f', 'Vars', {x,t});

else
    obs_funs = [];
    matlabFunction(navigation_funs.piecewise_f, 'File', './navigation/func/bump_f', 'Vars', {x});
    matlabFunction(navigation_funs.g, 'File', './navigation/func/rho_f', 'Vars', {x});
    matlabFunction(navigation_funs.density, 'File', './navigation/func/density_f', 'Vars', {x});
    matlabFunction(navigation_funs.grad_density, 'File', './navigation/func/grad_density_f', 'Vars', {x});
    matlabFunction(navigation_funs.hess_density, 'File', './navigation/func/hess_density_f', 'Vars', {x});
end

%% using Euler method
if(navigation_params.dynamic_obs)
    [x_euler, u_euler] = forwardEuler(@singleIntegrator_t,navigation_params);
else
    [x_euler, u_euler] = forwardEuler(@singleIntegrator,navigation_params);
end
x_euler = x_euler';

%% plot density
%fs = 1/euler_params.step_size;
%x_euler = lowpass(x_euler,100,fs);
plot_density(x_euler, navigation_params, obs_funs)

%% get plan with inverse kinematics
ee_plan.x = x_euler;
ee_plan.t = 0:euler_params.n_steps;
ee_plan.vel = u_euler;
%save('ee_plan');

joints = IK_planarRR(ee_plan.x);
time = ee_plan.t; % to do - run LQR before animate

%% animate
animate_planarRR(time,joints,navigation_params,obs_funs,ee_plan)

