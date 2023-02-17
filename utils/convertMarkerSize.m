function ax = convertMarkerSize(marker_size, traj_plot, axes)
%convertMarkerSize
% Convert marker size to the unit of your axes
% TODO: Verify that this works for higher dimensions (3+)
%
% Inputs:
%   marker_size         : Diameter of the marker size [units of axes]
%   traj_pts            : Chart line object (i.e. output of plot command
%                           used for plotting the marker)
%   axes                : Axes of the current figure
% Outputs:
%   ax                  : Axes of the current figure

ax = axes;
ax_units = get(ax, 'Units');
set(ax, 'Units', 'points'); % Set axis to points
ax_pos = get(ax, 'Position');
set(ax, 'Units', ax_units) % Reset to ax_units
xl = get(ax, 'XLim'); % Axis limits
markersize_pts = marker_size/(xl(2)-xl(1))*(ax_pos(3)-ax_pos(1));
set(traj_plot, 'MarkerSize', markersize_pts)


end