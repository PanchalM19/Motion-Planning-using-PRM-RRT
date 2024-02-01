% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    dist = sum((q_start - samples(1,:)).^2);
    origin = q_start;
    for a = 2:size(samples,1)
        dist_sub = sum((q_start - samples(a,:)).^2);
        if dist > dist_sub
            dist = dist_sub;
            origin = a;
        end
    end
    dist = sum((q_goal - samples(1,:)).^2);
    destin = q_goal;
    for a = 2:size(samples,1)
        dist_sub = sum((q_goal - samples(a,:)).^2);
        if dist > dist_sub
            dist = dist_sub;
            destin = a;
        end
    end
    roadmap= graph(adjacency);
    coll_free = shortestpath(roadmap, origin, destin);
    path = [];
    init  = 0;
    for a = coll_free
        path = [path; samples(a,:)];
        init = a;
    end    
    f_destin = [q_start];
    f_destin = [f_destin; path];
    f_destin = [f_destin; q_goal];
    path = f_destin;
    if init == destin
        path_found = true;
    else
        path_found = false;
    end
end