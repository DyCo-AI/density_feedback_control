function ee = FK_planarRR(joints)

% link lengths
robot_params = get_params();
a1 = robot_params.l1; a2 = robot_params.l2;

ee = [];
for i = 1:size(joints,1)
    % joint angles
    q1 = joints(i,1); q2 = joints(i,2);

    % end effector coordinates
    ee_x = a1*cos(q1) + a2*cos(q1+q2);
    ee_y = a1*sin(q1) + a2*sin(q1+q2);

    ee = [ee;[ee_x,ee_y]];

end
