function car_vertices = inflate(X)
% this function inflates the rear center of the ego vehicle into 
% four vertices based on car dimensions (!this is the vectorized version)
%
% input
% =====
% X:            ego vehicle state
%
% output
% ======
% car_vertices: four vertices of the ego vehicle with depth of n (amount of X)

    global param
    theta   = X(5,:);
    
    center  = X(1:2,:) + [param.lr*cos(theta); param.lr*sin(theta)];
    center = reshape(center, 2, 1, []);
    car_vertices = diag([param.len, param.wid])* [-0.5,  0.5, 0.5, -0.5;
                                                  -0.5, -0.5, 0.5,  0.5];
                                              
    car_vertices = repmat(car_vertices, 1,1, size(theta,2));
    
    for i = 1:size(theta,2)
       car_vertices(:,:,i) =  [cos(theta(i)), -sin(theta(i));
                               sin(theta(i)),  cos(theta(i))] * car_vertices(:,:,i);
    end
    
    car_vertices = car_vertices + center;
end