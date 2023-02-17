function [obstacle_bool, num_close_obs, close_obs_list] = localSensing(p, x_current, obstacle_names, current_obs_loc,...
    delta)
%localSensing
% Scan local range and determine if obstacle is within local sensing
% radius. If so and not in current obstacle list, returns obstacle_bool
% true that there is a obstacle w/in the local sensing range, number of 
% obstacle w/in local sensing range (num_close_obs), and the location of 
% the close obstacle (close_obs_list).
% Assumptions:
%   1) Assumes detect the center of the obstacles
%   2) Assumes all obstacles have same radius & are circles
%   3) No handle on probablistic information
%
% Remark:
%   Since location of trajectory may oscillate, 
%
% Inputs:
%   p               : Structure parameter containing local sensing range
%                       where field is named 'lsr'
%   x_current       : Current state (R^(nx1))
%   obstacle_names  : String array of all obstacle names (R^o)
%   current_obs_loc : List of current obstacle location stored in memory
%                       (R^(nxa))
%   delta           : Delta ball of percieved obstacle location vs. stored
%                       memory obstacle location to determine if obstacle
%                       has already been detected and stored (R^1_{>=0})
%
% Outputs
%   obstacle_bool   : Boolean that determines if obstacles not in list is
%                       within sensing radius
%   num_close_obs   : Number of obstacles w/in local sensing radius (R^d)
%   close_obs_list  : Location of obstacles w/in local sensing radius
%                       (R^(nxd))

% TODO: Make neater

if nargin < 5
    delta = 1e-3;
end

% Defining parameters
% Determine number of obstacles in memory
num_curr_obs = size(current_obs_loc,2);

% Determine number of obstacles
num_obstacles = size(obstacle_names,1);

local_range = p.('lsr');
rad_obs = p.('r1'); % Radius of obstacle

% Check curr_obs_loc if w/in local sensing range, if not, discard
num_curr_close_obs = num_curr_obs;


% For now, address when number of columns decrease/removed -> num columns
% < i -> Indexing done by i-col_rem_counter
% TODO: Make more intuitive / cleaner 
col_rem_counter = 0;
for i=1:num_curr_obs
    curr_dist_obs = norm(x_current - current_obs_loc(:,i-col_rem_counter)) - rad_obs; % Distance to obstacle boundary
    % If current obstacles leave local range
    if curr_dist_obs > local_range
        current_obs_loc(:,i-col_rem_counter) = []; % TODO: Optimize storing and removing obstacle location data
        col_rem_counter = col_rem_counter + 1;
        num_curr_close_obs = num_curr_close_obs - 1;
    end
end

% Initialize close obstacles
close_obs_list = current_obs_loc; % List of obstacles location w/in local sensing range
num_close_obs = num_curr_close_obs; % Number of obstacle w/in local sensing range

% Search if obstacles are within local sensing range
updated_obs_list = false; % Boolean for when new obstacle is w/in local sensing radius
for i=1:num_obstacles
    % Location of obstacles
    obs_loc = p.(sprintf("c%d", i));
    
    % Get distance of obstacle from current state
    dist_obs = norm(x_current - obs_loc) - rad_obs; % Distance to obstacle boundary
    
    % Check if obs distance within local range, if so -> Store location of
    % obstacle to update for density formulation
    if dist_obs <= local_range
        % Search if location of obstacle is not already stored in memory
        obs_in_memory = false;
        
        if num_close_obs > 0
            close_obs_different_counter = 0;
            % Search if current close obstacle location is in memory
            for j = 1:num_close_obs
                % If location of obstacle is not the same as obstacle in
                % memory, then add a counter to obstacle not in memory
                if norm(obs_loc - close_obs_list(:,j)) > delta
                    close_obs_different_counter = close_obs_different_counter + 1;
                end
            end
            
            % If counter is not the same as the length of the close
            % obstacle list in memory, then this obstacle is not unique
            % and update obs_in_memory to be true
            if close_obs_different_counter < size(close_obs_list,2)
                %fprintf("Obstacle in memory\n");
                obs_in_memory = true;
            end
        end
        
        if ~obs_in_memory
            %fprintf("Current obstacle w/in local sensing radius not in memory... updating list\n");
            close_obs_list = [close_obs_list, obs_loc]; % TODO: Optimize changing variable size
            num_close_obs = num_close_obs + 1;
            updated_obs_list = true;
        end
    end
end

if ~(num_close_obs && updated_obs_list)
    obstacle_bool = false;
else
    obstacle_bool = true;
    fprintf("Detected local obstacles... \n");
%     close_obs_list
%      num_close_obs
%      x_current
end

end

