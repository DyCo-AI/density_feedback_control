function [joint_obs, convex_hull] = gen_joint_obs(navigation_params)
% ref
% https://www.mathworks.com/matlabcentral/answers/1584644-intersection-of-circles-and-polygons
%% get parameters
[robot_params,~,~,~,animate_params] = get_params();
a1 = robot_params.l1 ; a2 = robot_params.l2;
w = 0.05; % width of robotic arm

% set these values for navigation params
% n = 1
% x_obs = [0.5, -0.5];
% x_obs_rad = 0.2;
n_obs = navigation_params.n_obs;
x_obs = navigation_params.x_obs;
x_obs_rad = navigation_params.x_obs_rad;

grayColor = [.7 .7 .7];


%% setup meshgrid
q1 = 0:0.1:2*pi;
q2 = 0:0.1:2*pi;
[Q1,Q2] = meshgrid(q1,q2);
Q1 = Q1(:);
Q2 = Q2(:);
joints = [Q1,Q2];
joint_obs = []; convex_hull = [];

%% update positions
for i = 1:length(joints)

    % get forward kinematics
    q1 = joints(i,1); q2 = joints(i,2);
    x1 = a1*sin(q1); y1 = -a1*cos(q1);
    x2 = x1 + a2*sin(q1+q2); y2 = y1 - a2*cos(q1+q2);
    
    clf()
    link1 = polyshape([x1+w*cos(q1) x1-w*cos(q1) -w*cos(q1) +w*cos(q1)], ...
    [y1+w*sin(q1) y1-w*sin(q1) -w*sin(q1) w*sin(q1)]);
    if(animate_params.plot_joint_obs)
        plot(link1,'FaceColor','red'); hold on
    end
    
    link2 = polyshape([x2+w*cos(q1+q2) x2-w*cos(q1+q2) x1-w*cos(q1+q2) x1+w*cos(q1+q2)], ...
    [y2+w*sin(q1+q2) y2-w*sin(q1+q2) y1-w*sin(q1+q2) y1+w*sin(q1+q2)]);
    if(animate_params.plot_joint_obs)
        plot(link2,'FaceColor','blue'); hold on
    end

    % plot circles for obstalces
    r1 = x_obs_rad(1); r2 = x_obs_rad(2);
    angles=linspace(0,2*pi,1000).'; angles(end)=[];
    circle1=polyshape([cos(angles), sin(angles)]*r1+[x_obs(1,1),x_obs(1,2)]);
    circle2=polyshape([cos(angles), sin(angles)]*r1+[x_obs(2,1),x_obs(2,2)]);
    if(animate_params.plot_joint_obs)
        plot(circle1,'FaceColor','black'); hold on;
        plot(circle2,'FaceColor','black'); hold on;
    end

    % plot iterations
    if(animate_params.plot_joint_obs)
        str = strcat("Iterations: " + num2str(i) + "/", num2str(length(joints)));
        text(0.8,-1.6,str,'HorizontalAlignment','left','VerticalAlignment','top');
        axis([-2.1 2.1 -2.1 2.1])
        pause(0.05);
    end    

    % find intersections
    V1=link1.Vertices; N1=size(V1,1);
    V1=V1([1:N1,1],:);
    V2=link2.Vertices; N2=size(V2,1);
    V2=V2([1:N2,1],:);
    for k = 1:N1
        intersect1=linexlines2D(circle1,V1(k,:),V1(k+1,:));
        intersect2=linexlines2D(circle1,V1(k,:),V1(k+1,:));
    end
    for j = 1:N2
        intersect3=linexlines2D(circle1,V2(j,:),V2(j+1,:));
        intersect4=linexlines2D(circle2,V2(j,:),V2(j+1,:));
    end

    if(~isempty(intersect1) || ~isempty(intersect2) ...
            || ~isempty(intersect3) || ~isempty(intersect4))
        %disp('intersect')
        joint_obs = [joint_obs; [q1,q2]];
    end  
    
end

if(animate_params.plot_joint_obs)
    figure()
    % plot robot and obs in task space
    p1 = subplot(1,2,1);
    q1 = pi; q2 = -pi/6;
    x1 = a1*sin(q1); y1 = a1*cos(q1);
    x2 = x1 + a2*sin(q1+q2); y2 = y1 + a2*cos(q1+q2);
    
    % plot robot
    link1 = polyshape([x1-w*cos(q1) x1+w*cos(q1) w*cos(q1) -w*cos(q1)], ...
    [y1+w*sin(q1) y1-w*sin(q1) -w*sin(q1) w*sin(q1)]);
    plot(link1,'FaceColor','red'); hold on
    link2 = polyshape([x2-w*cos(q1+q2) x2+w*cos(q1+q2) x1+w*cos(q1+q2) x1-w*cos(q1+q2)], ...
    [y2+w*sin(q1+q2) y2-w*sin(q1+q2) y1-w*sin(q1+q2) y1+w*sin(q1+q2)]);
    plot(link2,'FaceColor','blue'); hold on
                    
    % plot obs
    plot(circle1,'FaceColor','black'); hold on;
    plot(circle2,'FaceColor','black'); hold on;
    p1.XLim = [-2.1, 2.1]; p1.YLim = [-2.1, 2.1];
    xlabel('x'); ylabel('y');
    title('Obstalce represented in joint space');
    
    % plot obs in joint space
    p2 = subplot(1,2,2);
    scatter(joint_obs(:,1)',joint_obs(:,2)','MarkerEdgeColor',grayColor,...
          'MarkerFaceColor',grayColor,...
          'LineWidth',1.5); 
    k = boundary(joint_obs(:,1),joint_obs(:,2));
    convex_hull = [convex_hull; k];
    hold on;
    plot(joint_obs(k,1),joint_obs(k,2));

    p2.XLim = [-2*pi 2*pi]; p2.YLim = [-2*pi 2*pi];
    xticks(-2*pi:pi/3:2*pi); yticks(-2*pi:pi/3:2*pi);
    xtick = get(gca,'XTick'); ytick = get(gca,'YTick');
    set(gca, 'XTick', xtick,'XTickLabel',xtick.*180/pi)
    set(gca, 'YTick', ytick,'YTickLabel',ytick.*180/pi)
    xlabel('q1'); ylabel('q2');
    title('Obstalce represented in joint space');

    
end