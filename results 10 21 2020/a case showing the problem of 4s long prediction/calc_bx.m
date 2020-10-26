function [cx, cxx] = calc_bx(X, obstacle, i, ...
                                q1_back, q2_back, q1_front, q2_front, use_prediction)
% calc_bar_der: calculates first and second derivatives of the barrier
% function at a specified state X with finite difference estimation
%
% input
% =====
% X:        the state vector for the four-state model of the ego vehicle
% obstacle: instances of the obstacle class in a vector
% h:        finite difference step size
% q terms:  coefficients of the barrier functions, front and back denote
%           which center is used
% use_prediction: 0 to use deterministic model, 1 to use prediction
%
% output
% ======
% cx:       first derivative of the barrier function
% cxx:      second derivative of the barrier function

global param
global min_safe_dis%Omid
global Lane_size
global X_DIM U_DIM NUM_CTRL L dt
% global q1_back q2_back
% global q1_front q2_front

cx  = zeros(X_DIM,1);
cxx = zeros(X_DIM,X_DIM);

% formulate the ellipse 
if (use_prediction == 0)
    obs_pose    = obstacle.traj(:,i);... determinisitic model
    obs_theta   = obs_pose(5);
else
    obs_pose    = obstacle.traj_pred(:,i);... determinisitic model
    obs_theta   = obs_pose(3);%Omid: why is this 3?
end


obs_ellip   = obstacle.ellipse_axis(:,i);
ellip_a     = obs_ellip(1);
ellip_b     = obs_ellip(2);

% this vector is not full state difference
delX = [X(1) - obs_pose(1); X(2) - obs_pose(2)];

% this is the rotational matrix
rot_mat = [cos(obs_theta), -sin(obs_theta); 
           sin(obs_theta),  cos(obs_theta)];

% inequality constraint value
% intermediate matrix that is state independent
A   = rot_mat*diag([1/ellip_a^2, 1/ellip_b^2])*rot_mat';

%% the rear center of the ego vehicle
% scalar value inequality constraint
% 'if' added by Omid as we don't want min_safe_dis to mess up with the vehicles on other lanes 
if (abs(X(2) - obs_pose(2)) <= Lane_size/2)
    g   = 1+min_safe_dis - delX' * A * delX;   %min_safe_dis added by Omid
else
    g   = 1 - delX' * A * delX;   %min_safe_dis added by Omid

end

% second and third dimesion is not related, this is the derivative of the
% equality constraint
dg  = [-2* A' * delX;0; 0; 0];...     4x1 vector

% quadratize barrier function b(x)
db  = q1_back * q2_back * exp(q2_back * g) * (dg);              % scalar * (4x1) 
ddb = q1_back * q2_back ^2 * exp(q2_back * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

cx = cx + db;
cxx = cxx + ddb;

%% the front center of the ego vehicle
theta        = X(5);
x_front      = X + [cos(theta)*param.L; sin(theta)*param.L; 0;0; 0];

delX = [x_front(1) - obs_pose(1); x_front(2) - obs_pose(2)];


% if (X(1) > 32)
%     g=g;
% end

% if (abs(X(2) - obs_pose(2)) <= Lane_size/2)
% if (abs(X(1) - obs_pose(1)) <= 8)
%     g=g;
% end
% end

% 'if' added by Omid as we don't want min_safe_dis to mess up with the vehicles on other lanes 
if (abs(X(2) - obs_pose(2)) <= Lane_size/2)
    g   = 1 +min_safe_dis- delX' * A * delX;   %min_safe_dis added by Omid
else
    g   = 1 - delX' * A * delX;   %min_safe_dis added by Omid

end

% second and third dimesion is not related, this is the derivative of the
% equality constraint
dg  = [-2* A' * delX;0; 0; 0];...     4x1 vector

% quadratize barrier function b(x)
db  = q1_front * q2_front * exp(q2_front * g) * (dg);              % scalar * (4x1) 
ddb = q1_front * q2_front ^2 * exp(q2_front * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

cx = cx + db;
cxx = cxx + ddb;
end