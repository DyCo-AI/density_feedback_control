function navigation = get_density_joint(navigation_params, joint_obs)
%% Problem setup
% Some tidbit of information on parameter tuning.
% 1. If wish to have state converge to 0
%   a) Can increase number of simulated timesteps (N)
% 1. If wish to increase the rate at which the controller converges, can:
%   b) Change step size
%   c) Change alpha parameter
% 2. If wish to decrease the recoil from the abrupt gradients, decrease
%       the function value of g(x) and f(x) at obstacle (i.e. 0). This can
%       be done by:
%   a) Changing a (Changes the output value of f(x)*g(x))
%   b) Changing k (Directly shifts the graph up and down)
% 3. If wish to increase zeros in f(x) (for theoretical purposes), decrease
%       a.
%
% See this link for more intuition about how changes in parameter affect
% the curve of this function
% https://www.geogebra.org/calculator/uqfkbhva
%
% Remark: There are more ways how each parameters can affect the magnitude
% of control, convergence rate, and smoothness of control. Update these
% notes whenever you can.

% General function: f(x) = ke^(-a/(1-b(x-c)^2)) + d
% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
wrap = 0; % to wrap around the torous
tic
%% get parameters

x = sym('x',[2 1],'real'); % joint states
x_goal = navigation_params.x_goal;

%% form attracting density
% V(x):= 1/||x||^2*alpha
%density_params.alpha = 0.2; % Best value 0.2
%alpha = density_params.alpha;
%norm_x = sqrt((x(1)-x_goal(1))^2 + (x(2)-x_goal(2))^2);
%navigation.V = 1/(norm_x)^(2*alpha);

% V = 1/(1-cosq1)+(1-cosq2)^2
density_params.alpha = 1e-1; % Best value 0.2
alpha = density_params.alpha;
norm_x = sqrt((1-cos(x(1)-x_goal(1)))^2 + (1-cos(x(2)-x_goal(2)))^2);
navigation.V = 1/(norm_x)^(2*alpha);

%% Density Function Formulation for cirlces

% density_params.a = 1; 
% density_params.k = exp(density_params.a);
% density_params.sides1 = [1.5; 4.75]; % [lx; ly]
% density_params.sides2 = [1.5; 4.75]; % [lx; ly]
% density_params.sides3 = [1.5; 4.75]; % [lx; ly]
% density_params.sides4 = [1.5; 4.75]; % [lx; ly]
% density_params.b1  = density_params.sides1/2; % half length
% density_params.b2  = density_params.sides2/2; % half length
% density_params.b3  = density_params.sides3/2; % half length
% density_params.b4  = density_params.sides4/2; % half length

%c1 = [0.7;-0.1]; c2 = [4.75;-0.1];
%c3 = [1.6;2*pi-0.1]; c4 = [5.55;2*pi-0.1];
p = 2; bump = 1;

% best working
%obs 1
r1 = 0.5; r2 = 1; c1 = [0.7;1.1];
c = c1;
m = (norm((x-c), p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m)); vpa(f,2);
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m))); vpa(f_shift,2);
g = f/(f+f_shift); vpa(g,2);
bump = bump*g;

r1 = 0.5; r2 = 1.1; c1 = [1.3;0.1];
c = c1;
m = (norm((x-c), p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m)); vpa(f,2);
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m))); vpa(f_shift,2);
g = f/(f+f_shift); vpa(g,2);
bump = bump*g;

%obs2
r1 = 0.5; r2 = 1.3; c1 = [1.5;6.3];
c = c1;
m = (norm((x-c), p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m)); vpa(f,2);
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m))); vpa(f_shift,2);
g = f/(f+f_shift); vpa(g,2);
bump = bump*g;
% 
r1 = 0.5; r2 = 1; c1 = [2;5.5];
c = c1;
m = (norm((x-c), p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m)); vpa(f,2);
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m))); vpa(f_shift,2);
g = f/(f+f_shift); vpa(g,2);
bump = bump*g;


navigation.piecewise_f = bump;

%% get final density = rho * bump
navigation.density = navigation.V * navigation.piecewise_f;
% navigation.density = navigation.piecewise_f;

navigation.grad_density = gradient(navigation.density, x);
%navigation.hess_density = hessian(navigation.density, x);

toc
end