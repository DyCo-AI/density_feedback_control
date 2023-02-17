function plot_density_joint(x_euler,navigation_params,obs_funs)
[~,euler_params] = get_params();
%% parameters
load('joint_obs.mat');
N = euler_params.n_steps;
dT = euler_params.step_size;
grayColor = [.7 .7 .7];

x_goal = navigation_params.x_goal;
n_obs = navigation_params.n_obs;
%x_obs = navigation_params.x_obs;
%x_obs_rad = navigation_params.x_obs_rad;
x_obs = [0.5, -0.5];
x_obs_rad = 0.2;

%% -----------------------------------------------------------------------------------------
%                           Plotting for static obstacles
% -----------------------------------------------------------------------------------------
%% Plotting Density Function
x = -2*pi:0.1:2*pi;
y = x;
[X,Y] = meshgrid(x,y);

Z = zeros(size(X));
Z_grad_x1 = zeros(size(X));
Z_grad_x2 = zeros(size(X));
for i=1:length(X)
    for j = 1:length(Y)
        Z(i,j) = density_f([x(j);y(i)]);
        z_grad = grad_density_f([x(j);y(i)]);
        Z_grad_x1(i,j) = z_grad(1);
        Z_grad_x2(i,j) = z_grad(2);
    end
end
figure()
subplot(2,2,[1,3])
p1 = gca;
surf(X,Y,Z, 'FaceAlpha',0.65, 'EdgeColor', 'none')
colormap jet
view(90,0)
xlabel('q1'); ylabel('q2');
title("Density Function")

%% plot gradients
% figure(1)
% subplot(2,2,2)
% p2 = gca;
% quiver(X, Y, Z_grad_x1, Z_grad_x2,3);
% xlabel('q1'); ylabel('q2');
% title("Gradient of Density Function")

%% plot x,y trajectory in 2d
subplot(2,2,[2,4])
p3 = gca;
scatter(joint_obs(:,1)',joint_obs(:,2)','MarkerEdgeColor',grayColor,...
          'MarkerFaceColor',grayColor,...
          'LineWidth',1.5);
p3.XLim = [0 2*pi]; p3.YLim = [0 2*pi];
xticks(0:pi/3:2*pi); yticks(0:pi/3:2*pi);
% xtick = get(gca,'XTick'); ytick = get(gca,'YTick');
% set(gca, 'XTick', xtick,'XTickLabel',xtick.*180/pi)
% set(gca, 'YTick', ytick,'YTickLabel',ytick.*180/pi)
xlabel('q1'); ylabel('q2');
title('Obstalce represented in joint space');
hold on

% if joint obs is approximated as rectangle
% sides = [2.7; 6.2];
% c = [4.05;3.1];
% for n = 1:n_obs
%     r = sides; p = c-(r/2);  
%     rectangle('Position',[p', r'],'FaceColor','none','EdgeColor',grayColor);
%     hold on;
% end
contour(X,Y,Z,'LevelStep',0.2); hold on
plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
plot(x_euler(:,1),x_euler(:,2),'k-', 'LineWidth', 2); hold on;
plot(x_goal(1),x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;

legend('obstacles','Start','Trajectory','Goal')
title('Joint Space Obstacle Avoidance');

xlabel('q1')
ylabel('q2')
hold off


end