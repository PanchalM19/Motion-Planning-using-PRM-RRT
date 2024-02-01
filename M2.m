% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    samples = []; %Empty matrix to append later.
    while length(samples)< num_samples
        q_sort = (q_max-q_min).*rand(1,4) + q_min;
        coll_free = (all(q_sort>=q_min) && all(q_sort<= q_max));
        collision = check_collision(robot, q_sort,link_radius, sphere_centers, sphere_radii);
        if coll_free && ~collision
            samples = [samples ; q_sort]; 
        end
    end
    adjacency = zeros(num_samples, num_samples); %zeros matrix to append later
    for a = 1:length(samples)
        select_sum = sum((repmat(samples(a,:),length(samples),1) - samples).^2, 2); 
        [sorted index] = sort(select_sum)
        neigh(1,:) = index(1:num_neighbors);
        for b = neigh(1,:)
            if adjacency(a,b) > 0
                continue;
            end    
            collision = check_edge(robot, samples(a,:), samples(b,:), link_radius, sphere_centers, sphere_radii);
            if collision
                adjacency(a,b) = 0;
            else 
                adjacency(a,b) = sqrt(sum(((samples(a,:) - samples(b,:)).^2)));
                adjacency(b,a) = sqrt(sum(((samples(a,:) - samples(b,:)).^2)));
            end
        end
    end
    save("M2");
end