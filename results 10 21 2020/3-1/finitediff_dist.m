function db = finitediff_dist(X, obstacle, t, h)
% finitediff_bar: calculates the derivative of the closest distance
% function at a single time step
%
% input
% =====
% X: nominal position of the center of the vehicle
% h: finite difference intervals
% obstacle: 1 obstacle
% t: time step t
%
% output
% ======
% db: gradient of the closest distance function

if (nargin < 4)
    h = 2^(-10);
end
% calculate f(x+h)
X_forward   = repmat(X,1,4) + h*diag([1 1 1 1]);
car_vert = inflate(X_forward);
dist_for = calc_nearest_dist(car_vert, obstacle.vertices(:,:,t));

% calculate f(x)
car_vert = inflate(X);
dist = calc_nearest_dist(car_vert, obstacle.vertices(:,:,t));

db = (dist_for - dist)./h;
db = reshape(db, 4, []);

end