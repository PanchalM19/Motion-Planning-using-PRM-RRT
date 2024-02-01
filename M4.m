% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    function pt_dist = dis2pt(a,b)
        pt_dist = sqrt(sum((a-b).^2,2));
    end
    origin = (q_start); 
    init = []; 
    % Initializing the parameters.
    beta = 0.1;
    alpha = 0.03;
    l_node = 0;
    destin = [];
    ite = 3000; 
    ver = 1; 
    for a=1:ite
        if rand(0,1) < beta  
            destin = q_goal;
        else
            destin = M1(q_min, q_max,1);
        end
        distance = sqrt(sum((repmat(destin,ver,1)-origin).^2,2));
        [~, q_i] = sort(distance);
        near_pt = origin(q_i(1),:);
        pt_index = q_i(1);
        new = near_pt + ((alpha./dis2pt(destin, near_pt)) .* (destin - near_pt));
        [~, ind] = ismember(new,origin, 'rows');
        if ind ~=1
            if ~check_collision(robot, new, link_radius, sphere_centers, sphere_radii)
                 origin = [origin; new];
                ver = ver + 1;
                init = [init; [pt_index, ver, dis2pt(new,near_pt)]];
            end
        end
        if abs(destin-near_pt) < 0.001
            final_pt = ver;
            break;
        end
    end
    if (l_node==0)
        dist2 = sum((repmat(q_goal,length(origin),1) - origin).^2,2);
        [~, q_ind2] = sort(dist2);
        final_pt = q_ind2(1:1);
    end
    final_path = shortestpath(graph(init(:,1), init(:,2), init(:,3)), 1, final_pt);
    path = (q_start);
    if ~isempty(final_path)
        path_found = true;
        for item = final_path
            path = [path; origin(item,:)];
        end
    end
end