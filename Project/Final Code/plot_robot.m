function [] = plot_robot(pose,fig)
    % This function plots a graphical robot.
    % This function is borrowed from the robotarium code
    % available at robotarium.gatech.edu
    % It uses GRITSBOT_PATCH which is a helper function to generate patches for the simulated GRITSbots.

    x=pose(1,1);
    y=pose(2,1);
    th=pose(3,1)- pi/2;

    %Initialization
    % Initialize variables
    offset = 0.05;
    boundaries = [-5, 5, -5, 5]; 

    % Plot Space boundaries
    b = boundaries;
    boundary_points = {[b(1) b(2) b(2) b(1)], [b(3) b(3) b(4) b(4)]};
    boundary_patch = patch('XData', boundary_points{1}, ...
        'YData', boundary_points{2}, ...
        'FaceColor', 'none', ...
        'LineWidth', 3, ...,
        'EdgeColor', [0, 0, 0]);

    set(fig, 'color', 'white');

    % Set axis
    ax = fig.CurrentAxes;

    % Limit view to xMin/xMax/yMin/yMax
    axis(ax, [boundaries(1)-offset, boundaries(2)+offset, boundaries(3)-offset, boundaries(4)+offset])
    set(ax, 'PlotBoxAspectRatio', [1 1 1], 'DataAspectRatio', [1 1 1])

    % Store axes
    axis(ax, 'off')            

    % Static legend
    setappdata(ax, 'LegendColorbarManualSpace', 1);
    setappdata(ax, 'LegendColorbarReclaimSpace', 1);           

    % Apparently, this statement is necessary to avoid issues with
    % axes reappearing.
    hold on

        data = gritsbot_patch;
        robot_body = data.vertices;

        rotation_matrix = [
            cos(th) -sin(th) x;
            sin(th)  cos(th) y;
            0 0 1];
        transformed = robot_body*rotation_matrix';
        robot_handle = patch(...
            'Vertices', transformed(:, 1:2), ...
            'Faces', data.faces, ...
            'FaceColor', 'flat', ...
            'FaceVertexCData', data.colors, ...
            'EdgeColor','none');
    %Draw Robot
    set(robot_handle, 'Vertices', transformed(:, 1:2));
    drawnow limitrate
    
end     