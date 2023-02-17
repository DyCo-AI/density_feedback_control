function navigation = get_density(navigation_params)
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

tic

density_params.a = 0.1; % Positive definite | Best value 0.1
density_params.k = -exp(density_params.a); % Negative definite | k = -exp(a) s.t bump function asymptotically @ 1

for j = 1:navigation_params.n_obs
    % Change radius: decrease b -> increase radius % If parameter d, d >= 1
    % params.radius = 1/sqrt(params.b)
    % params.b = 1/(params.radius)^2
    density_params.b(j) = 1/navigation_params.x_obs_rad(j)^2;
end

% Function: g(x):= 1/||x||^alpha
density_params.alpha = 0.2; % Best value 0.2
%% Density Function Formulation
% Create function of density and bump function
x = sym('x',[2 1],'real');

navigation.piecewise_f = -density_params.k/exp(density_params.a);
for j=1:navigation_params.n_obs
    % Obstacle function
    c = navigation_params.x_obs(j,:)';
    r = navigation_params.x_obs_rad(j);
    f(j) = density_params.k/exp(density_params.a/(1-((norm(x-c)^2)/r^2))) - density_params.k/exp(density_params.a);

    % Bump function rho(x) = g(x)*f(x)
    navigation.piecewise_f = piecewise(...
       (norm(x-c)^2)/r^2<1, f(j), navigation.piecewise_f);
end
                    
navigation.g = 1/norm(x-navigation_params.x_goal)^(2*density_params.alpha); % rho

% Density function (rho * bump)
navigation.density = navigation.g * navigation.piecewise_f;

navigation.grad_density = gradient(navigation.density, x);
navigation.hess_density = hessian(navigation.density, x);

toc
end