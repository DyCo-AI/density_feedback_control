function plot_density(x_euler, navigation_params,obs_funs)
[~,euler_params,~,~,animate_params] = get_params();
%% parameters
N = euler_params.n_steps;
dT = euler_params.step_size;
grayColor = [.7 .7 .7];

x_goal = navigation_params.x_goal;
n_obs = navigation_params.n_obs;
x_obs = navigation_params.x_obs;
x_obs_rad = navigation_params.x_obs_rad;

if animate_params.flag_movie
        name = ['animations/plannarRR_dynamic.mp4'];
        vidFile = VideoWriter(name,'MPEG-4');
        vidFile.FrameRate = 5;
        open(vidFile);
end
skip_rate = animate_params.skip_rate; %skip this many frames per loop

%% -----------------------------------------------------------------------------------------
%                           Plotting for dynamic obstacles
% -----------------------------------------------------------------------------------------
if(animate_params.dynamic_obs.density_gif)
    figure
    plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    plot(x_goal(1),x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    
    for n = 1:n_obs
        obs_t = obs_funs{n}(0);
        %viscircles([obs_t(1), obs_t(2)],params.x_obs_rad(n));
        r = x_obs_rad(n); d = 2*r; 
        px = obs_t(1)-r; py = obs_t(2)-r;  
        obs(n) = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor',grayColor);
    end
 
    plot(x_euler(1:end,1),...
        x_euler(1:end,2), 'black', 'LineWidth', 2);
    hold on;
    legend('Start','Goal','Trajectory', 'Location', 'southwest')
    xlabel('x1');  ylabel('x2')
    xlim([-3,3]); ylim([-3,3]);
    hold on
    
    t = 1:size(x_euler,1);
    figure
    plot(t,x_euler)
    xlabel("Iteration")
    ylabel("States")

% Obstacle Avoidance Gif for dynamic trajectory 
    % jj := frame #
    for jj=1:skip_rate:N
        f1 = figure(999);
        clf(f1); % close current figure
        plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); 
        hold on;
        plot(x_goal(1),x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); 
        hold on;
        
        for n = 1:n_obs
            obs_t = obs_funs{n}(jj*dT)';
            %viscircles([obs_t(1), obs_t(2)],params.x_obs_rad(n)); 
            r = x_obs_rad(n); d = 2*r; 
            px = obs_t(1)-r; py = obs_t(2)-r;  
            obs(n) = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor',grayColor);
            hold on;
        end
    
        plot(x_euler(jj,1), x_euler(jj,2), 'black*', 'LineWidth', 3); 
        hold on;
        legend('Start','Goal','traj', 'Location', 'southwest')
        
        xlabel('x1'); ylabel('x2')
        xlim([-10,10]); ylim([-10,10]);
        
        timestamp = sprintf("Time: %0.2f s", dT*jj*skip_rate);
        text(0.6,0.1, timestamp, 'Units', 'normalized');
        obs_period = sprintf("Obstacle frequence: %0.2f Hz", params.obstacle_freq);
        text(0.6, 0.05, obs_period, 'Units', 'normalized');
        
        if animate_params.flag_movie
            writeVideo(vidFile, getframe(gcf));
        end
    end
    hold off
end
% dynamic obstacles plot 2
% plot only static traj plot for dynamic obstacle case
if(animate_params.dynamic_obs.traj)
    disp('plot static traj for dynamic obstacle case')

    figure
    plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    plot(x_goal(1),x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    
    for n = 1:n_obs
        obs_t = obs_funs{n}(0);
        %viscircles([obs_t(1), obs_t(2)],params.x_obs_rad(n));
        r = x_obs_rad(n); d = 2*r; 
        px = obs_t(1)-r; py = obs_t(2)-r;  
        obs(n) = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor',grayColor);
    end
    plot(x_euler(1:end,1), x_euler(1:end,2), 'black', 'LineWidth', 2);
    legend('Start','Goal','Trajectory', 'Location', 'southwest')
    xlabel('x1'); ylabel('x2')
    xlim([-3,3]); ylim([-3,3]);
    hold off
end

%% -----------------------------------------------------------------------------------------
%                           Plotting for static obstacles
% -----------------------------------------------------------------------------------------

if(animate_params.static_obs)
    %% Plotting Density Function
    [X,Y] = meshgrid(-10:0.2:10, -10:0.2:10);
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
    figure()
    surf(X,Y,Z, 'FaceAlpha',0.65, 'EdgeColor', 'none')
    colormap jet
    view(90,0)
    title("Density Function")
    
    %% plot gradients
    figure()
    quiver(X, Y, Z_grad_x1, Z_grad_x2,3);
    title("Gradient of Density Function")
    
    %% plot x,y trajectory in 2d
    figure()
    for n = 1:n_obs
        %viscircles(params.x_obs(j,:),params.x_obs_rad(j)); hold on;
        r = x_obs_rad(n); d = 2*r; 
        px = x_obs(n,1)-r; py = x_obs(n,2)-r;  
        rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor',grayColor);
        hold on;
    end
    plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    plot(x_euler(:,1),x_euler(:,2),'black', 'LineWidth', 2); hold on;
    plot(x_goal(1),x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    
    legend('Start','Trajectory','Goal')
    title(['Solution using $\rho(x) = \frac{-e^{||x-c||^2}+1}{||x||}$ ... ' ...
        'and $u_i = \frac{\partial \rho}{ \partial x_i}$'],'Interpreter','Latex');
    
    xlabel('x1')
    ylabel('x2')
    axis equal
    hold off
end

if animate_params.flag_movie
    close(vidFile);
end

end