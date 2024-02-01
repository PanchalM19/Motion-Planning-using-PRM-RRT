% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    init = 0;
    smoothed_path = path(1,:);
    for a = 1:size(path, 1)
        for b = a+1:size(path, 1)
            coll = check_edge(robot, path(a,:), path(b,:), link_radius, sphere_centers, sphere_radii);
            if ~coll
                init = b;
            end
        end
        if init ~=a
         smoothed_path = [smoothed_path; path(init,:)];
         a = init;
        end
    end
    smoothed_path = unique(smoothed_path, "rows", "stable");
    %I used unique to identify the uncommon keypoints
end