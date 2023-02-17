function animate_planarRR_joint(t,joints,navigation_params)
%% get robot params
[robot_params,euler_params,~,~,animate_params] = get_params();

%% setup gif
figure()
flag_movie = animate_params.flag_movie;
skip_rate = animate_params.skip_rate; %skip this many frames per loop

if flag_movie
    try
        name = ['animations/planarRR_joint.mp4'];
        vidfile = VideoWriter(name,'MPEG-4');
    catch ME
        name = ['/animations/planarRR_joint'];
        vidfile = VideoWriter(name,'Motion JPEG AVI');
    end
    open(vidfile);
end

%% get parameeters
a1 = robot_params.l1 ; a2 = robot_params.l2;
dT = euler_params.step_size;

x_ini = FK_planarRR(navigation_params.x_ini);
x_goal = FK_planarRR(navigation_params.x_goal);
n_obs = navigation_params.n_obs;
x_obs = navigation_params.x_obs;
x_obs_rad = navigation_params.x_obs_rad;

N = length(t)-1;
w = 0.05; % width of robotic arm

%% update positions
for i = 1:skip_rate:N

    % get forward kinematics
    q1 = joints(i,1); q2 = joints(i,2);
    x1 = a1*sin(q1); y1 = a1*cos(q1);
    x2 = x1 + a2*sin(q1+q2); y2 = y1 + a2*cos(q1+q2);
    
    clf()

    % plot starting position of end effector
    plot(x_ini(1),x_ini(2),'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on
    
    % plot goal position on the end effector
    plot(x_goal(1),x_goal(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); 

    link1 = polyshape([x1-w*cos(q1) x1+w*cos(q1) w*cos(q1) -w*cos(q1)], ...
    [y1+w*sin(q1) y1-w*sin(q1) -w*sin(q1) w*sin(q1)]);
    plot(link1,'FaceColor','red'); hold on
    
    link2 = polyshape([x2-w*cos(q1+q2) x2+w*cos(q1+q2) x1+w*cos(q1+q2) x1-w*cos(q1+q2)], ...
    [y2+w*sin(q1+q2) y2-w*sin(q1+q2) y1-w*sin(q1+q2) y1+w*sin(q1+q2)]);
    plot(link2,'FaceColor','blue'); hold on

    % plot circles for obstalces
    for n = 1:n_obs
        r = x_obs_rad(n);
        angles=linspace(0,2*pi,1000).'; angles(end)=[];
        circle=polyshape([cos(angles), sin(angles)]*r+[x_obs(n,1),x_obs(n,2)]);
        plot(circle,'FaceColor','black');
    end

    % plot iterations
    T = t*dT;
    T_end = t(end)*dT;
    str = strcat("Time: " + num2str(T) + " s" + "/", num2str(T_end+" s"));
    timestamp = text(1.5,-2.5,str,'HorizontalAlignment','left','VerticalAlignment','top');
    set(timestamp,'String',str);
    axis([-2.1 2.1 -2.1 2.1])
    pause(0.05);

end

if flag_movie
    close(vidfile);
end
end