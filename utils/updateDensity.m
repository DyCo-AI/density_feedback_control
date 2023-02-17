function updated = updateDensity(x, params, obs_loc)
%updateDensity
% Update density function given obstacle location
%
% Inputs:
%   x               : Symbolic variable (R^(nx1))
%   params          : Structure parameter containing navigation parameters
%   obs_loc         : List of obstacle location to update density (R^(nxd))
% 
% Outputs:
%   updated         : Boolean if density updated

% TODO: Get the lyapunov measure systematically
% TODO: Get update density to work for fastInvBump
% Right now, problem with updated obstacles redundantly

tic
% Get number of obstacles
num_obs = size(obs_loc,2);



% Form bump function
bump = 1;
if ~params.enable_fast_inv_bump
    for i=1:num_obs
        bump = bump*formPNormBump(params.r1, params.r2, obs_loc(:,i), x, params.p, true);
    end
else
    bump = formFastInvBump(x, params, num_obs);
end

% Generate density function
g = 1/norm(x-params.xd)^(2*params.alpha);
density = g*bump;
grad_density = gradient(density);
%hess_density = hessian(density);

matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x}, 'Optimize', false);
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x}, 'Optimize', false);
%matlabFunction(hess_density, 'File', 'functions/hess_density_f', 'Vars', {x}, 'Optimize', false);

fprintf("Updated density function\n");
toc

end

