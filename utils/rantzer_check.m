function [rantzer_cond, pos_not_rantz_cond, rantzer_fig] = rantzer_check(rantzer_check, rantzer_graph, grid_size)
% rantzer_check
%   Checks numerically through a grid search if Rantzer's condition is satisfied
%   Currently only checks the function density_f. Future works will check a
%   user specified function else defaults to checking the density function.
%   Currently, works with 2D control density functions at the moment...
%   Future works makes this more general
% 
% Input:
%   rantzer_check       : Boolean variable to check rantzer's condition
%   rantzer_graph       : Boolean variable to output graph figure handle
%   grid_size           : Resolution size for grid search of rantzer's
%                           condition (Default: 0.1)
% Output:
%   rantzer_cond        : Boolean result if rantzer's condition was
%                           satisfied
%   pos_not_rantz_cond  : Vector containing the position where rantzer's
%                           condition is not satisfied
%   rantzer_fig         : Figure handle of graphical representation of
%                           rantzer's condition

% Check Rantzer's Condition Graphically | div (rho u)

% Remark: Maybe some bug for now... Hessian gives NaN's.... could either be
%   0s or infinity (density_f([0;0]) = NaN which should be inf). Therefore
%   adding to NaN does not work...
% Last remark:
%   Testing out if gradient(1/t^2) @ t=0 -> Gives inf... so perhaps b/c
%   original function is not easy that it gives NaN instead...

addpath(pwd + "\functions\")

if (nargin < 3)
    grid_size = 0.1;
end

if (nargin < 2)
    rantzer_graph = false;
end

if (nargin < 1)
    rantzer_check = true;
end

% Output variables
rantzer_cond = [];
rantzer_fig = [];
pos_not_rantz_cond = [NaN, NaN]; % Initialize to "false" (i.e. rantz cond satisfied)

x_vec = -10:grid_size:10; y_vec = x_vec;
[X,Y] = meshgrid(x_vec, y_vec);
rho_val = zeros(size(X));
grad_rho_x1 = zeros(size(X));
grad_rho_x2 = zeros(size(X));

laplacian_rho_x1 = zeros(size(X));
laplacian_rho_x2 = zeros(size(X));

for i=1:length(x_vec)
    for j = 1:length(y_vec)
        rho_val(i,j) = density_f([x_vec(j);y_vec(i)]);
        grad_rho = grad_density_f([x_vec(j);y_vec(i)]);
        grad_rho_x1(i,j) = grad_rho(1);
        grad_rho_x2(i,j) = grad_rho(2);
        
        laplacian_rho = diag(hess_density_f([x_vec(j);y_vec(i)])); % Specific for 2d rn
        laplacian_rho_x1(i,j) = laplacian_rho(1);
        laplacian_rho_x2(i,j) = laplacian_rho(2);
    end
end


if rantzer_check
    % Get |doh(rho)/doh(x)|^2 (i.e. doh(roh)/doh(x)^T doh(rho)/doh(x))
    grad_rho_squared = grad_rho_x1.^2 + grad_rho_x2.^2;
    
    % Get rho * laplacian_rho
    rho_laplacian_rho = rho_val.*(laplacian_rho_x1 + laplacian_rho_x2);
    
    rantzer_cond = rho_laplacian_rho + grad_rho_squared;
    if(any(rantzer_cond<0, 'all'))
        fprintf("Warning.... There exists a rantzer condition value less than 0\n");
        
        % Find position where rantzer condition not satisfied
        [ix_not_rantz_cond, iy_not_rantz_cond] = find(rantzer_cond < 0);
        for i=1:length(ix_not_rantz_cond)
            pos_not_rantz_cond(i,:) = [X(1,ix_not_rantz_cond(i)), Y(iy_not_rantz_cond(i), 1)];
        end
        
    else
        fprintf("Rantzer condition satisfied through search-based methods\n");
    end
    
    if rantzer_graph
        rantzer_fig = figure();
        surf(X,Y, rantzer_cond, 'FaceAlpha', 0.65, 'EdgeColor', 'none');
        colormap jet
        view(90,0)
        %title("Rantzer Condition")
        view(10, 20);
        xlabel("x_1", 'FontSize', 20)
        ylabel("x_2", 'FontSize', 20)
        % view([-5 -5 10])
        zlim([-5 10])
        hold on
        if (any(rantzer_cond < 0, 'all')) % For now just checking when condition  less than 0
            scatter3(pos_not_rantz_cond(:,2), pos_not_rantz_cond(:,1), zeros(size(pos_not_rantz_cond, 1), 1),...
                5, 'MarkerFaceColor',[1 0 0], 'MarkerEdgeColor', [1 0 0]);
            lgd = legend("$\nabla \cdot (\rho u)$", "$\nabla \cdot (\rho u) < 0$", "Interpreter", "latex",...
                'Location', 'south');
            lgd.FontSize = 14;
        end
    end
    
    
end