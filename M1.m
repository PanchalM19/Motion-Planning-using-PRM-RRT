% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    %q1 min&max are the joint limits.
    %we need to uniformly distribute num_sampples between q_min and q_max.
    qs = zeros(num_samples, 4); %create an empty qs to append later
    for a = 1:4
        qs(:,a)=q_min(a)+(q_max(a)-q_min(a))*rand(num_samples,1)
        
        % Don't know what was wrong in the below method.
%         unif_dist = linspace(q_min(a),q_max(a),length(num_samples))
%         qs(:,a) = interp1(unif_dist, num_samples, length(qs));
    end
end