function [single_int_p, dbl_int_p, dynamics_p] = generateStateSpace(deltaT, x)
%generateStateSpace
%   Generates the state space matrices and (currently) the feedback LQR
%   control gain for each system
% Input:
%   deltaT              : Discrete timem step
%   x                   : Symbolic variable to determine size
% Output:               : Optional parameter to define discrete time step
%                           (Default: 0.01)
%   single_int_p        : Stucture variable containing the single
%                           integrator state space model and LQR gain
%   dbl_int_p           : Structure variable containing the double
%                           integrator state space model and LQR gains

% TODO(AZ): MAKE ROBUST FOR HIGHER DIMENSION

if (nargin < 1)
    deltaT = 0.01;
end

if (nargin < 2)
    n = 2;
else
    [n, ~] = size(x);
end

if nargin < 3
    u_dim = n;
end

%% Single Integrator
single_int_p.u_dim = u_dim; % Assumes u on xddot
single_int_p.A = zeros(n,n);
single_int_p.B = eye(n);
single_int_p.Q = 10*eye(n); % Good value: 10
single_int_p.R = 100*eye(n); % Original value: 25

single_int_p.K = lqr(single_int_p.A, single_int_p.B, ...
                    single_int_p.Q, single_int_p.R);

%% Double Integrator
dbl_int_p.u_dim = u_dim; % Assumes control on xddot -> half the state dimension
dbl_int_p.A = zeros(2*n, 2*n); 
dbl_int_p.A(1:n, n+1:2*n) = eye(n);

dbl_int_p.B = zeros(2*n, n); % Double integrator -> dim p = dim n

dbl_int_p.B(n+1:2*n,:) = eye(n);
dbl_int_p.Q = 10*eye(2*n); % 2nx2n | Original Value: 10
dbl_int_p.R = 10*eye(n); % Original Value: 100
dbl_int_p.K = lqr(dbl_int_p.A, dbl_int_p.B, ...
                        dbl_int_p.Q, dbl_int_p.R);
                    
dbl_int_c = ss(dbl_int_p.A,dbl_int_p.B,...
            eye(length(dbl_int_p.A)),0); % Continuous time form)
dbl_int_d = c2d(dbl_int_c, deltaT); % Discrete time form


dbl_int_p.A_d = dbl_int_d.A; dbl_int_p.B_d = dbl_int_d.B;
dbl_int_p.Q_d = dbl_int_p.Q*deltaT; dbl_int_p.R_d = dbl_int_p.R*deltaT;

dbl_int_p.K_d = lqrd(dbl_int_p.A, dbl_int_p.B, ...
                        dbl_int_p.Q, dbl_int_p.R, deltaT);
                    
%% General Dynamics
dynamics_p.u_dim = 2; % State space control dimension
dynamics_p.x_dim = 4; % State space dim

% EoM
% M(x)xddot + C(x,xdot)xdot + G(x) = bu
dynamics_p.M = [1 0;0 1]; % Inertial matrix
dynamics_p.C = [0 0;0 0]; % Coriolis Terms
dynamics_p.G = [0;0]; % Nonlinearity due to potential field(doh V/ doh q)
% TODO: H = C(x,xdot)*xdot + G(x)
dynamics_p.H = [0;0]; % Nonlinearity terms (Gyroscopic & coriolis acceleration)
dynamics_p.B_hat = diag([1;1]);


end