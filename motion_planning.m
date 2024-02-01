% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 0 and 6 that denotes question
%                       number to run.
%        samples, adjacency -> Optional pre-computed PRM samples and
%                              adjacency matrix to avoid re-computing it in
%                              question M3
% Output: samples, adjacency -> If PRM roadmap was computed, you can save
%                               it and pass it in on later calls to
%                               hw2_motion to avoid re-computing it

function [samples, adjacency] = ex2_motion(questionNum, samples, adjacency)

    close all;
    
    if nargin < 1
        error('Error: Please enter a question number as a parameter');
    end

    % Create robot
	robot = create_robot();

    % Start and goal configuration
    q_start = [0 -pi/4 0 -pi/4];
    q_goal = [0 -3 0 -3];
    % Minimum and maximum joint angles for each joint
    q_min = [-pi/2 -pi 0 -pi];
    q_max = [pi/2 0 0 0];
    % Radius of each robot link's cylindrical body
    link_radius = 0.03;
    
    % Set up spherical obstacle
    sphere_center = [0.5 0 0];
    sphere_radius = 0.25;

    % Plot robot and obstacle
    robot.plot(q_start);
    hold on;	
    draw_sphere(sphere_center,sphere_radius);

    % ========== Question M0 ==========
    if questionNum == 0
        % Plot the robot in a given configuration
        q = M0();
        robot.plot(q);
        % Check if configuration is within joint limits / in collision
        in_bounds = (all(q >= q_min) && all(q <= q_max));
        in_collision = check_collision(robot, q, link_radius, sphere_center, sphere_radius);
        if in_bounds
            disp('Robot configuration is within joint limits.');
        else
            disp('Robot configuration is not within joint limits.');
        end
        if in_collision
            disp('Robot configuration is in collision.');
        else
            disp('Robot configuration is not in collision.');
        end
    end
    
    % ========== Question M1 ==========
    if questionNum == 1
        num_samples = 100;
        % Draw random samples from configuration space, within joint limits
        % TODO: Implement this function
        random_qs = M1(q_min, q_max, num_samples);
        num_in_bounds = 0;
        num_collision = 0;
        for i = 1:num_samples
            q = random_qs(i,:);
            robot.plot(q);
            % Check if sampled configuration is within bounds
            % Note: All samples should be within bounds
            if all(q >= q_min) && all(q <= q_max)
                num_in_bounds = num_in_bounds + 1;
            end
            % Check if sampled configuration is in collision
            % Note: Samples can be in collision
            if check_collision(robot, q, link_radius, sphere_center, sphere_radius)
                num_collision = num_collision + 1;
            end
        end
        fprintf('Number of samples: %d\n', num_samples);
        fprintf('Number within bounds: %d\n', num_in_bounds);
        fprintf('Number in collision: %d\n', num_collision);
    end

    % ========== Question M2 ==========
    if questionNum == 2
        % Parameters for PRM
        num_samples = 100;
        num_neighbors = 10;
        % Construct the roadmap, consisting of
        % configuration samples and weighted adjacency matrix
        % TODO: Implement this function
        [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_center, sphere_radius);
        figure;
        % Visualize weighted adjacency matrix
        imshow(adjacency, [min(min(adjacency)), max(max(adjacency))]);

%         [samples, adjacency] = hw3_motion(2);
    end
    
    % ========== Question M3 ==========
    if questionNum == 3
        % Parameters for PRM
        num_samples = 100;
        num_neighbors = 5;
        % If pre-computed roadmap is not provided,
        % compute the roadmap using M2
        if nargin < 3
            [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_center, sphere_radius);
        end
        % Use the roadmap to find a path from q_start to q_goal
        % TODO: Implement this function
        [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_center, sphere_radius);
        % Visualize the trajectory, if one is found
        if path_found
            fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
            disp(path);
            robot.plot(interpolate_path(path), 'fps', 10);
        else
            disp('No path found.');
        end
    end
    
    % ========== Question M4 ==========
    if questionNum == 4
        % Use the RRT algorithm to find a path from q_start to q_goal
        % TODO: Implement this function
        [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius);
        % Visualize the trajectory, if one is found
        if path_found
            fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
            disp(path);
            robot.plot(interpolate_path(path), 'fps', 10);
        else
            disp('No path found.');
        end
    end
    
    % ========== Question M5 ==========
    if questionNum == 5
        % Use the RRT algorithm to find a path from q_start to q_goal
        [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_center, sphere_radius);
        if path_found
            fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
            disp(path);
            % If trajectory is found, smooth the trajectory
            % TODO: Implement this function
            smoothed_path = M5(robot, path, link_radius, sphere_center, sphere_radius);
            % Visualize the smoothed trajectory
            fprintf('Smoothed path found with %d intermediate waypoints:\n', size(smoothed_path, 1) - 2);
            disp(smoothed_path);
            robot.plot(interpolate_path(smoothed_path), 'fps', 10);
        else
            disp('No path found.');
        end
    end
    
    % ========== Question M6 ==========
    if questionNum == 6
        % Set up a more challenging motion planning problem
        % If M4 (RRT) and M5 (smoothing) are implemented correctly,
        % this should find a trajectory without any issues
        r = 0.38;
        % Create more spherical obstacles
        sphere_centers = [sphere_center; 0 0.5 0; 0 -0.5 0];
        sphere_radii = [sphere_radius; r; r];
        for i = 2:size(sphere_centers, 1)
            draw_sphere(sphere_centers(i,:)', sphere_radii(i));
        end
        % Use the RRT algorithm to find a path from q_start to q_goal
        [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii);
        if path_found
            fprintf('Path found with %d intermediate waypoints:\n', size(path, 1) - 2);
            disp(path);
            % If trajectory is found, smooth the trajectory
            smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii);
            % Visualize the smoothed trajectory
            fprintf('Smoothed path found with %d intermediate waypoints:\n', size(smoothed_path, 1) - 2);
            disp(smoothed_path);
            robot.plot(interpolate_path(smoothed_path), 'fps', 10);
        else
            disp('No path found.');
        end
    end
end

% ========== Helper functions ==========

% Given a path consisting of configuration waypoints,
% interpolates the waypoints to give a less abrupt trajectory.
% Consecutive pairs of waypoints are interpolated along a straight line
% in configuration space, where the newly generated points are
% less than max_velocity apart in configuration space.
% This helper function is primarily used for visualization purposes.
function trajectory = interpolate_path(path, max_velocity)
    if nargin < 2
        max_velocity = 0.05;
    end
    trajectory = [path(1,:)];
    for i = 2:size(path, 1)
        vec = path(i,:) - path(i-1,:);
        num_ticks = ceil(norm(vec) / max_velocity);
        ticks = linspace(0, 1, num_ticks + 1)';
        segment = repmat(path(i-1,:), num_ticks, 1) + repmat(ticks(2:end,:), 1, length(path(i,:))) .* repmat(vec, num_ticks, 1);
        trajectory = [trajectory; segment];
    end
end

% Create a 4-DOF arm with 2 links
function robot = create_robot()
    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);    
    robot = SerialLink(L, 'name', 'robot');
end

% Draw a sphere with specified center position and radius
function draw_sphere(position, radius)
    [X,Y,Z] = sphere;
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    X = X + position(1);
    Y = Y + position(2);
    Z = Z + position(3);
    surf(X,Y,Z);
end