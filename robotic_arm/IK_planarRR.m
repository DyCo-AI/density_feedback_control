function joints = IK_planarRR(ee)

% get robot params
robot_params = get_params();

% link lengths
a1 = robot_params.l1; a2 = robot_params.l2;

% using geometric approach
% https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
% (ee -> joints) mapping is not unique
% cos q2 is symmetric about zero
% hence 2 solutions for q2 is posisble (+/- q2)
% taking q2 is positive always

joints = [];
for i = 1:size(ee,1)
% end effecotr positions angles
x = ee(i,1); y = ee(i,2);
    
    q2 = acos((x^2+y^2-a1^2-a2^2)/2*a1*a2);

    if(imag(q2))
        disp('goal is outside the reachable space of robot')
    end

    q1 = atan2(y,x) - atan2(a2*sin(q2),(a1+a2*cos(q2))) + pi/2;
    j = [q1, q2];

joints = [joints; j];

end
