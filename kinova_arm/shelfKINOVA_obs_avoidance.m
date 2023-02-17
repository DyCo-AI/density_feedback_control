clc; clear; close all;
% ref
% https://www.mathworks.com/help/supportpkg/robotmanipulator/ug/track-trajectory-inverse-kinematics.html
%% load parameters from plan
load('planarKINOVA_ee_plan.mat')
load('planarKINOVA_navigation_params.mat')

%% obstacle parameters


%% generate robot and get tf
r = load('exampleHelperKINOVAGen3GripperCollRRT.mat'); 
gen3 = r.robot;
gen3.DataFormat = 'column';

q_home = [0 15 180 -130 0 55 90]'*pi/180;
eeName = 'EndEffector_Link';
T_home = getTransform(gen3, q_home, eeName);
T_home(1:4,4) =  [0;0;0;1];

%% Visualize the robot
figure()
ax = show(gen3,q_home,'Frames','off');
axis auto;
view([-50, 20]); 

%% generate planes
plane = collisionBox(1.5,1.5,0.05);
plane.Pose = trvec2tform([0.25 0 -0.025]);
hold on
plane.show('Parent', ax);

% obstacle
obs = collisionBox(0.25, 0.25, 0.5);
obs.Pose = trvec2tform([0.6 0.25 0.25]);
[~, patchObj] = obs.show('Parent', ax);
patchObj.FaceColor = [1 0 0];

% Visualize shelves
%generate shelf 1
W = 0.305;
sx = 0.2;
sy = 0.4+W/2;
sz = 0;
fv = stlread('exampleHelperShelves.stl');
shelfB = trisurf(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
shelf = hgtransform;
shelfB.Parent = shelf;
shelf.Matrix = trvec2tform([sx sy sz]);

%shelf 2
W = 0.305;
sx = 0.2;
sy = -0.7+W/2;
sz = 0;
fv = stlread('exampleHelperShelves.stl');
shelfB = trisurf(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
shelf = hgtransform;
shelfB.Parent = shelf;
shelf.Matrix = trvec2tform([sx sy sz]);
drawnow;
%% create Inverse kinematics
ik = inverseKinematics('RigidBodyTree',gen3);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1, 1, 1, 1, 1, 1];
q_init = q_home;

%% define waypoints for ee traj default
plan_filt = filter_ee_plan();
points = [plan_filt.x + 0.1, plan_filt.y, 0.2.*ones(length(plan_filt.x),1)];

%% display waypoints in a plot
hold on;
plot3(points(:,1),points(:,2),points(:,3),'-*g', 'LineWidth', 1.5);
xlabel('x');
ylabel('y');
zlabel('z');
axis auto;
view([60,10]);
grid('minor');

%% solve inverse kinematics at each waypoint
numJoints = size(q_home,1);   
numWaypoints = size(points,1);
qs = zeros(numWaypoints,numJoints);
for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:)';
    %T_des = trvec2tform(points(i,:)) * eul2tform([pi/2 0 pi/2]);
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
  
    
    % Store the configuration
    qs(i,:) = q_sol(1:numJoints); 
    
    % Start from prior solution
    q_init = q_sol;
end
%% Animate
figure; 
set(gcf,'Visible','on');
set(gcf, 'renderer', 'zbuffer');
ax = show(gen3,qs(1,:)','Frames','off');
ax.CameraPositionMode='auto';
view([60,10]); 
hold on;
 
% set up environment
plane = collisionBox(1.5,1.5,0.05);
plane.Pose = trvec2tform([0.25 0 -0.025]);
hold on
plane.show('Parent', ax);

% target
% rightWidget = collisionBox(0.05, 0.05, 0.05);
% rightWidget.Pose = trvec2tform([0.3 0.65 0.05]);
% [~, patchObj] = rightWidget.show('Parent', ax);
% patchObj.FaceColor = [1 0 0];

% obstacle
Cylinder1 = collisionCylinder(safe_rad, obs_height);
Cylinder1.Pose = trvec2tform(obs_location);
[~, patchObj] = Cylinder1.show('Parent', ax);
patchObj.FaceColor = [1 0 0];
patchObj.EdgeColor = "none";
hold on

Cylinder2 = collisionCylinder(safe_rad+0.05, obs_height);
Cylinder2.Pose = trvec2tform(obs_location);
[~, patchObj] = Cylinder2.show('Parent', ax);
patchObj.FaceColor = [0 1 0];
patchObj.FaceAlpha = 0.3;
patchObj.EdgeColor = "none";
hold on

% Plot waypoints
plot3(points(:,1)+0.1,points(:,2),points(:,3),'-g','LineWidth',2);
axis auto;
view([-30,25]); 
grid('minor');
hold on;
 
title('Simulated Movement of the Robot');
% Animate
framesPerSecond = 30;
r = robotics.Rate(framesPerSecond);
for i = 1:numWaypoints
    show(gen3, qs(i,:)','PreservePlot',false, 'Frames','off');
    drawnow;
    waitfor(r);
end
