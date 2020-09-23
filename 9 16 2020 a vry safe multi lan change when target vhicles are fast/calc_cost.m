function cost = calc_cost(X, U, ref_traj, obstacle, use_prediction, stopping)
% calc_cost: calculates the cost for the entire trajectory
%
% input
% =====
% X:        ego vehicle trajectory          4 x (NUM_CTRL+1)
% U:        ego vehicle control sequence    4 x (NUM_CTRL)
% ref_traj: reference trajectory            4 x (NUM_CTRl+1)
% obstacle: objects of obstacle class in a vector
% use_prediction: 0 to use deterministic model, 1 to use prediction
%
% output
% ======
% cost:     scalar cost

% global paramerters
global X_DIM NUM_CTRL param
global q1_back q2_back q1_front q2_front q1_road q2_road q1_stop q2_stop
global x_stop
global q1_jerk q2_jerk q1_acc q2_acc q1_del q2_del
global a_dot_max a_dot_min v_dot_max v_dot_min delta_max delta_min
global w_ref w_vel w_jerk w_acc w_del w_end_ref w_end_acc w_end_vel
% road upper and lower limits
global road_up_lim road_low_lim Lane_size
global q1_CenterLane q2_CenterLane %added by Omid
global v_min q1_min_vel q2_min_vel%added by Omid


%for tailgate
global min_safe_dis

% load cost_weights
cost_weights;

% define barreir function terms
% % log barrier 
% global d_safe 
% global t_bar
% bar     = @(d) -1/t_bar * log(d - d_safe);

% exponential barrier
% bar     = @(d, q1, q2) q1*exp(q2*(d_safe - d));

% initialize things
cost = 0.0;
Xref = zeros(X_DIM, NUM_CTRL + 1);

% main loop for calculating costs
for i = 1:1:NUM_CTRL
    Xi = X(:,i);
    Ui = U(:,i);
    % reference trajectory
    [~, index] = min(sum((ref_traj' - Xi).^2));
    Xref(:,i) = transpose(ref_traj(index,:));
    
    % obstacle term, exponential barrier
    for j = 1:length(obstacle)
        % formulate the ellipse 
        if use_prediction == 0
            obs_pose    = obstacle(j).traj(:,i);
            obs_theta   = obs_pose(5);
        else
            obs_pose    = obstacle(j).traj_pred(:,i);
            obs_theta   = obs_pose(3);
        end

        obs_ellip   = obstacle(j).ellipse_axis(:,i);
        ellip_a     = obs_ellip(1);
        ellip_b     = obs_ellip(2);

        % this vector is not full state difference
        delX = [Xi(1) - obs_pose(1); Xi(2) - obs_pose(2)];

        % this is the rotational matrix
        rot_mat = [cos(obs_theta), -sin(obs_theta); 
                   sin(obs_theta), cos(obs_theta)];

        % inequality constraint value
        % intermediate matrix that is state independent
        A   = rot_mat*diag([1/ellip_a^2, 1/ellip_b^2])*rot_mat';

        %% the rear center of the ego vehicle
        % scalar value inequality constraint
%         g   = 1 - delX' * A * delX;   
        % 'if' added by Omid as we don't want min_safe_dis to mess up with the vehicles on other lanes 
        if (abs(Xi(2) - obs_pose(2)) <= Lane_size/2)
            g   = 1 +min_safe_dis- delX' * A * delX;   %min_safe_dis added by Omid
        else
            g   = 1 - delX' * A * delX;   %min_safe_dis added by Omid
       end
        % exponential barrier function
        cost = cost + q1_back * exp(q2_back * g);
        
        %% the front center of the ego vehicle
        X_front = Xi + [cos(Xi(5))*param.L; sin(Xi(5))*param.L; 0;0; 0];
        delX = [X_front(1) - obs_pose(1); X_front(2) - obs_pose(2)];
        % scalar value inequality constraint
%         g   = 1 - delX' * A * delX;   
        % 'if' added by Omid as we don't want min_safe_dis to mess up with the vehicles on other lanes 
        if (abs(X_front(2) - obs_pose(2)) <= Lane_size/2)
            g   = 1 +min_safe_dis- delX' * A * delX;   %min_safe_dis added by Omid
        else
            g   = 1 - delX' * A * delX;   %min_safe_dis added by Omid
       end
        % exponential barrier function
        cost = cost + q1_front * exp(q2_front * g);
    end
    
    % ============== road costraints ==============
    cost = cost + q1_road * exp(q2_road * (Xi(2) - road_up_lim));
    cost = cost + q1_road * exp(q2_road * (road_low_lim - Xi(2)));
    
    % ==================== penalizing distanc from the lane center (Omid) ====================
    CenterLaneY = CenterLaneY_detector(Xi(2));
    g   = abs(Xi(2)-CenterLaneY) - 1;
    cost = cost + q1_CenterLane * exp(q2_CenterLane * g) ;      
    
     % ================ other state limits (Omid)===================
    %min velocity constraint
    g   =v_min-Xi(4);
    cost = cost + q1_min_vel * exp(q2_min_vel * g);

    % acceleration
    % upper bound
    g   = Xi(3) - v_dot_max;
    cost = cost + q1_acc * exp(q2_acc * g);
    
    % lower bound 
    g   = v_dot_min - Xi(3);
    cost = cost + q1_acc * exp(q2_acc * g);
%     plot(cost,Xi(2));
    % ================ ctrl limits ===================
    % jerk
    % upper bound
    g   = Ui(1) - a_dot_max;
    cost = cost + q1_jerk * exp(q2_jerk * g);
    
    % lower bound 
    g   = a_dot_min - Ui(1);
    cost = cost + q1_jerk * exp(q2_jerk * g);
    
    % steering
    % upper bound
    g   = Ui(2) - delta_max;
    cost = cost + q1_del * exp(q2_del * g);
    
    % lower bound 
    g   = delta_min - Ui(2);
    cost = cost + q1_del * exp(q2_del * g);
    
    % ============== stop sign ===============
    if (stopping == 1)
        cost = cost + q1_stop * exp(q2_stop * (Xi(1) - x_stop));
    end
    
%     % obstacle term log barrier
%     ego_vertices = inflate(Xi);
%     dist = 2^10;
%     for j = 1:length(obstacle)
%         obs_vertices = obstacle(j).vertices;
%         dist = min(d2poly(ego_vertices, obs_vertices(:,:,i)));
%     end  
%     cost = cost + w_bar * bar(dist);
% vectorize state for cost calculation 
cost = cost + [w_ref(i) w_ref(i) w_acc(i) w_vel(i) 0]*(Xi-Xref(:,i)).^2;%it is tested and it's correct Omid
end

% fill in the end point for the reference trajectory
Xref(:,end) = transpose(ref_traj(end,:));
%  Xref(:,end) =[];
%  X(:,end) =[];

% vectorize state for cost calculation 
% cost = cost + [w_ref(end) w_ref(end) w_acc w_vel(end) 0]*(X(:,end)-Xref(:,end)).^2;%added by Omid to get the cost of last step of trajectory 
cost = cost + [w_end_ref w_end_ref w_end_acc w_end_vel 0]*(X(:,end)-Xref(:,end)).^2;%added by Omid to get the cost of last step of trajectory 

% cost = cost + sum([w_ref w_ref w_acc w_vel 0]*(X-Xref).^2);%edited by Omid
% cost = cost + sum([1.0130 1.0130 w_acc 6.9094 0]*(X-Xref).^2);

% vectorize control inputs cost
cost = cost + sum([w_jerk(i) w_del(i)]*U.^2);%edited by Omid

% stop sign
if (stopping == 1)
    cost = cost + q1_stop * exp(q2_stop * (X(end, 1) - x_stop));
end

% % obstacle cost for end point
% ego_vertices = inflate(X(:, end));
% 
% for j = 1:size(obstacle)
%     obs_vertices = obstacle(j).vertices;
%     for k = 1:size(ego_vertices, 2)
%         dist = min(d2poly(ego_vertices, obs_vertices(:,:,end)));
%     end    
% end  
% cost = cost + w_bar * bar(dist);

end