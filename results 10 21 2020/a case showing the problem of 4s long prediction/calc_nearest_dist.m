function dist = calc_nearest_dist(ego_vert, target_vert)
% calc_nearest_dist: calculate the nearest distance given the ego vehicle
% rectangular vertices and the polygon representing the obstacle
%
% input
% =====
% ego_vert: 2x4xk, ego vehicle vertices
% target_vert: 2xn
% 
% output
% ======
% dist: closest distance between two convex hull size of k

k = size(ego_vert, 3);
dist = 10000*ones(1,k);

for i = 1:k
    dist(i) = min(dist(i), d2poly(ego_vert(:,:,i),target_vert));
end

end