% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q -> 1x4 vector denoting the configuration to check for collision
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: in_collision -> Boolean, true if the robot at configuration q is
%                         in collision with the given spherical obstacles

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, resolution)
    x1 = [0 0 0]'; %Initial position of the robot. (x1 is the origin of link 1)
    T2 = robot.A(1,q) * robot.A(2,q) * robot.A(3,q);
    x2 = T2.t; % x2 is where link 1 and link 2 are connected.
    T3 = T2 * robot.A(4,q);
    x3 = T3.t; %x3 is the end of the link 2.
    
    if nargin < 6
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1);
    % repmat helps to describe the locations of the copies of A. No of
    % copies is 1...n.
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    points = [x12 x23];
    
    in_collision = false;
    % for loop checks for collision with the sphere.
    for i = 1:size(sphere_centers, 1) 
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break; % if collision is found, the if statement is exited and the collision is displayed.
        end
    end
end