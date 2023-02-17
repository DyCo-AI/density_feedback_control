function dynamics = dynamics_planarRR(t, x, u, robot_params)
m1 = robot_params.m1 ; m2 = robot_params.m2;
l1 = robot_params.l1 ; l2 = robot_params.l2;
g = robot_params.g;
I1 = robot_params.I1 ;I2 = robot_params.I2; 
lc1 = l1/2; lc2 = l2/2;


q1 = x(1,:); q2 = x(2,:); dq1 = x(3,:); dq2 = x(4,:);

m11 = I1 + I2 + m2*l1^2 + 2*m2*l1*lc2.*cos(q2);
m12 = I2 + m2*l1*lc2.*cos(q2);
m22 = I2*ones(1,size(x,2));
dynamics.M = [m11 m12; m12 m22];

dynamics.C = [-2*m2*l1*lc2.*sin(q2).*dq2.*dq1-m2*l1*lc2.*sin(q2).*dq2.*dq2;
     m2*l1*lc2.*sin(q2).*dq1.*dq1+zeros(1, size(x,2)).*dq2];

% G is negative for q1 = 0  about vertical
% G is positive for q1 = pi about vertical
dynamics.G = [m1*g*lc1.*sin(q1) + m2*g.*(l1.*sin(q1)+lc2.*sin(q1+q2));
     m2*g*lc2.*sin(q1+q2)];


dq = [dq1; dq2]; 

% for acrobot
%B = [zeros(1,size(x,2)); ones(1,size(x,2))];
%ddq = M\(B.*u-C-G);

B = eye(2);
ddq = dynamics.M\(B*u-dynamics.C-dynamics.G);

dynamics.f = [dq;ddq];
end