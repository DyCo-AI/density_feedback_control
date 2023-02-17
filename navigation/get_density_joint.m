function navigation = get_density_joint(navigation_params, joint_obs, convex_hull)
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
% density_params.a = 1;  % Positive definite | Best value 0.1
% density_params.k = -exp(density_params.a);
% density_params.rad = 0.3;
% density_params.b = 1/density_params.rad^2;
% 
% % Create bump function
% navigation.piecewise_f = -density_params.k/exp(density_params.a);
% for j=1:length(joint_obs)
%     % Obstacle function
%     c = joint_obs(j,:)';
%     r = density_params.rad;
%     k = density_params.k;
%     a = density_params.a;
%     f(j) = k/exp(a/(1-((norm(x-c)^2)/r^2))) - 1;
% 
%     % Bump function rho(x) = g(x)*f(x)
%     %norm_xc = sqrt((x(1)-c(1))^2 + (x(2)-c(2))^2);
%     navigation.piecewise_f = piecewise(...
%        (norm(x-c)^2)/r^2<1, f(j), navigation.piecewise_f);
% end            
%% Density Function Formulation for squares
density_params.a = 1; 
density_params.k = exp(density_params.a);
density_params.sides1 = [1.5; 4.75]; % [lx; ly]
density_params.sides2 = [1.5; 4.75]; % [lx; ly]
density_params.sides3 = [1.5; 4.75]; % [lx; ly]
density_params.sides4 = [1.5; 4.75]; % [lx; ly]
density_params.b1  = density_params.sides1/2; % half length
density_params.b2  = density_params.sides2/2; % half length
density_params.b3  = density_params.sides3/2; % half length
density_params.b4  = density_params.sides4/2; % half length
density_params.c1 = [0.7;-0.1];
density_params.c2 = [4.75;-0.1];
density_params.c3 = [1.6;2*pi-0.1];
density_params.c4 = [5.55;2*pi-0.1];

% define square bump 1 and 2
for i=1:length(x)
    c1 = density_params.c1; c2 = density_params.c2;
    c3 = density_params.c3; c4 = density_params.c4;
    r1 = density_params.b1; r2 = density_params.b2;
    r3 = density_params.b3; r4 = density_params.b4;
    k = density_params.k; a = density_params.a;

    f1 = -(k/exp(a/(1-(x(i)-c1(i))^2/r1(i)^2))) + 1;
    f2 = -(k/exp(a/(1-(x(i)-c2(i))^2/r2(i)^2))) + 1;
    f3 = -(k/exp(a/(1-(x(i)-c3(i))^2/r3(i)^2))) + 1;
    f4 = -(k/exp(a/(1-(x(i)-c4(i))^2/r4(i)^2))) + 1;
end

% get domain for the square bump1
k = density_params.k;
a = density_params.a;

for i=1:length(x)
    if (i==1)
        navigation.piecewise_f1 = piecewise(...
            norm(x(i)-c1(i)) < r1(i), f1, k/a);
    else
        navigation.piecewise_f1 = piecewise(...
            norm(x(i)-c1(i)) < r1(i), navigation.piecewise_f1, k/a);
    end
end

% get domain for the square bump2
for i=1:length(x)
    if (i==1)
        navigation.piecewise_f2 = piecewise(...
            norm(x(i)-c2(i)) < r2(i), f2, navigation.piecewise_f1);
    else
        navigation.piecewise_f2 = piecewise(...
            norm(x(i)-c2(i)) < r2(i), navigation.piecewise_f2, navigation.piecewise_f1);
    end
end

% get domain for the square bump3
for i=1:length(x)
    if (i==1)
        navigation.piecewise_f3 = piecewise(...
            norm(x(i)-c3(i)) < r3(i), f3, navigation.piecewise_f2);
    else
        navigation.piecewise_f3 = piecewise(...
            norm(x(i)-c3(i)) < r3(i), navigation.piecewise_f3, navigation.piecewise_f2);
    end
end

% get domain for the square bump4
for i=1:length(x)
    if (i==1)
        navigation.piecewise_f4 = piecewise(...
            norm(x(i)-c4(i)) < r4(i), f4, navigation.piecewise_f3);
    else
        navigation.piecewise_f4 = piecewise(...
            norm(x(i)-c4(i)) < r4(i), navigation.piecewise_f4, navigation.piecewise_f3);
    end
end

navigation.piecewise_f = navigation.piecewise_f4;

%% get final density = rho * bump
navigation.density = navigation.V * navigation.piecewise_f;
%navigation.density = navigation.V;
navigation.grad_density = gradient(navigation.density, x);
navigation.hess_density = hessian(navigation.density, x);

toc
end