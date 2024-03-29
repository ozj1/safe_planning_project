clear all; close all; clc;

%% Description
% CiLQR - solve the deterministic finite-horizon optimal control problem.
%
%        minimize (sum_i CST(x(:,i),u(:,i))) + CST(x(:,end))
%            u
%        s.t.  x(:,i+1) = DYN(x(:,i),u(:,i))
%
% State and Control Space
% =======================
% X - the state space is 4-dimensional
%     X(1): x coordinate of the center of the rear axle of the vehicle
%     X(2): y coordinate of the center of the rear axle of the vehicle
%     X(3): velocity of the vehicle
%     X(4): heading angle of teh vehicle
%
% U - the control space is 2-dimensional
%     U(1): acceleration (v_dot)
%     U(2): steering angle
%
% Functions Used in This File
% ===========================
% forward_pass():   obtain the trajectory by performing forward kinematics
% backward_pass():  computes k, K
% DYNCST():         calculate cx, cxx, cu, cuu, cux
% calc_cost():      calculates cost
% reftraj_gen():    helper funtion used to generate reference trajectory

%% parameters declaration
% --- vehicle parameters
vehicle_parameter;
cost_weights;

global L
L = param.lf + param.lr;

% --- LQR loop parameters
tolFun         = 0.1;...            reduction exit criterion
maxIter        = 20;...             maximum iterations            
lambda         = 0.5;...            initial value for lambda
dlambda        = 1;...              initial value for dlambda
lambdaFactor   = 1.6;...            lambda scaling factor
lambdaMax      = 1e10;...           lambda maximum value
lambdaMin      = 1e-6;...           below this value lambda = 0
alpha          = 0.5;...            line search step size

% --- planning horizon parameters
global T
global T_horizon
global dt           
global t_switch

T = 30;             ... total job length%Omid it was 20 previously
T_horizon = 4;      ... sliding horizon length
dt = 0.1;           ... discretization time
t_switch = 0.5;     ... 

global NUM_CTRL;                    ... number of controls
global NUM_TOTAL;                   ... number of sliding window
NUM_TOTAL = round(T / dt);
NUM_CTRL  = round(T_horizon / dt);  

% --- state dimension
global X_DIM 
X_DIM = 5; 

% --- input dimension
global U_DIM
U_DIM = 2;

% --- environment 
global d_safe
d_safe = 15;

global x_stop
x_stop = 75;

% to update trajectory generation
global y_final y_temp_final y_temp_init traj_count


% road upper and lower limitstraj_count
global road_up_lim road_low_lim Lane_size
global vref aref vref_road

 % for obstacle overtaking until the road is clear 
 global EgoPolicy Phase

% --- initialize trace data structure
trace = struct('iter',nan,'lambda',nan,'dlambda',nan,'cost',nan);
trace = repmat(trace,[maxIter 1]);
trace(1).iter = 1;
trace(1).lambda = lambda;
trace(1).dlambda = dlambda;

%% initializaion
% --- reference trajectory and simulation environment generation
vref        = 15;
vref_road=15;
aref=0.;
traj_count=0;
ego_initial_y=-Lane_size/2.;
scenario_ref='Linear';%'LaneChange';
%vehicle objects
egoV=vehicle;
vehicle_type='Ego';%to make sure global variables of y_final and y_temp_final will update only for case of Ego vehicle 
Phase=1;EgoPolicy=0.;%the initial ref traj policy mackers 
% ref_traj_fractions=0*ones(2.*NUM_TOTAL+1,5);
%specfing th final destintion of go vehicle
y_init=ego_initial_y;
        if (y_init == -Lane_size)
            y_final = Lane_size;
        elseif (y_init == Lane_size)
            y_final = Lane_size;
        elseif (y_init == Lane_size/2.)
            y_final = Lane_size/2.;
        elseif (y_init == -Lane_size/2.)
            y_final = Lane_size/2.;
        elseif (y_init == -3*Lane_size/2.)
            y_final = 3*Lane_size/2.;
        elseif (y_init == 3*Lane_size/2.)
            y_final = 1*Lane_size/2.;
        end
tf=1;%seconds needed to do the lane change 
ref_traj    = reftraj_gen(2*T, dt,ego_initial_y ,20,scenario_ref , aref,vref,tf,0,vehicle_type);%it must be 3 for D1 and -3 for others 
top_lane    = reftraj_gen(2*T, dt,  road_up_lim,20, 'Linear', aref,vref,tf,0, 'road_line');
side_lane   = reftraj_gen(2*T, dt, road_low_lim,20, 'Linear', aref, vref,tf,0,'road_line');
left_road   = reftraj_gen(2*T, dt, road_up_lim,20, 'Linear', aref, vref,tf,0,'road_line');
right_road  = reftraj_gen(2*T, dt, road_low_lim,20, 'Linear', aref, vref,tf,0,'road_line');
left_road_half   = reftraj_gen(2*T, dt, road_up_lim/2,20, 'Linear', aref, vref,tf,0,'road_line');
right_road_half  = reftraj_gen(2*T, dt, road_low_lim/2,20, 'Linear', aref, vref,tf,0,'road_line');
mid_road    = reftraj_gen(2*T, dt,  0,20, 'Linear', aref, vref,tf,0,'road_line');

% ref_traj    = reftraj_gen(2*T, dt, 3, 'Linear', aref, vref);%it must be 3 for D1 and -3 for others 
% top_lane    = reftraj_gen(2*T, dt,  3, 'Linear', aref, vref);
% side_lane   = reftraj_gen(2*T, dt, -3, 'Linear', aref, vref);
% left_road   = reftraj_gen(2*T, dt,  road_up_lim, 'Linear', aref, vref);
% right_road  = reftraj_gen(2*T, dt, road_low_lim, 'Linear', aref, vref);
% mid_road    = reftraj_gen(2*T, dt,  0, 'Linear', aref, vref);

% --- initial state ---
X_start     = zeros(X_DIM, 1);
X_start(2)  = ego_initial_y;   ... initial y offset of -3m (on the ref trajectory)%it must be 3 for D1 and -3 for others 
X_start(4)  = 15;   ... initial velocity of 14.0m/sec

% --- goal state ---
X_goal      = transpose(ref_traj(end,:));
% X_goal      = [300;2;15;0];%added by Omid to get to the location we want

% --- initial control sequence
U0      = zeros(U_DIM, NUM_CTRL); 
U0(1,:) = 0.00 * ones(1,NUM_CTRL);      ... initial acceleration sequence
U0(2,:) = 0.00 * ones(1, NUM_CTRL);     ... initial steering sequence

% --- set up scenarios ---
%   D1 -- one target vehicle cuts in, obstacle going from -3 to 3
%   A1 -- one front target vehicle travels slowly around 9m/s (overtake)
%   A2 -- one front target vehicle travels fast around 12m/s (ACC)
%   A3 -- one front target vehicle traverls slowly, one target vehicle 
%           incoming in the opposite lane (safe overtake while merging
%           back) y0 = 3m
%   A4 -- one front target vehicle to follow and lane changing 
%           to an open lane. need to change the starting position of x
%   A5 -- one front target vehicle to follow and lane changing 
%           to the next lane with target vehicle in front
%   A6 -- one front target vehicle to follow and lane changing 
%           to the next lane with target vehicles front and behind 
%   O1--one front target vehicle (v=8m/s) to follow and lane changing 
%           to the next lane with target vehicles behind (10m/s) go vehicle 15m/s
scenario = 'Od6';
[OBS, cut_in_traj] = scenario_generation(scenario);
tgt_reach_traj = cut_in_traj;... this is used by reachability analysis

stop_sign = 0;... use A2 to test the stop sign scenario

% --- prediction mode parameters
use_prediction = 1;...              0 to use deterministic model
                   ...              1 to use prediction results
use_reach      = 1;...              0 to not use reach model
                   ...              1 to use reach
use_adaptive   = 1;...              0 to not use adaptive weight tuning
                   ...              1 to use adaptive weight tuning
               
% --- two obstacle test cases ---
% % obstacle1
% o1_init = [50;3;5;0];
% o1_vert = inflate(o1_init);
% o1      = obstacle(o1_init(1:2), o1_vert, o1_init(3), o1_init(4), NUM_CTRL, dt, 1);
% o1      = o1.update_traj(false);
% 
% % obstacle2 
% o2_init = [120;-3; -8;0];
% o2_vert = inflate(o2_init);
% o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), NUM_CTRL, dt, 1);
% o2      = o2.update_traj(false);
% 
% obstacle = [o1, o2];

% --- obtain initial trajectory
[X, U] = forward_pass(U0, [], [], [], X_start, dt, []);

%% Main Loop
% planned trajectory recording
X_planned = zeros(X_DIM, NUM_TOTAL);
U_planned = zeros(U_DIM, NUM_TOTAL);

% variables for visualization
obs_traj    = [];
X_local     = [];
obs_short   = [];
obs_long    = [];
loop_time   = [];


% iLQR
for i = 1:1:NUM_TOTAL
    

    
    %we have a two phases planner: phase 1 for  for obstacle overtaking until the road is clear then phase 2 for going to the desired destination if we're not on it already 
    % ref generation for obstacle overtaking until the road is clear 
    if i>NUM_CTRL/5 && i<NUM_TOTAL-NUM_CTRL/5
    traj_dy_dist = zeros(NUM_CTRL/5,1);
           sum_traj_dy_dist=0;
            for jj = i-NUM_CTRL/5:1:i-1
                traj_dy_dist(jj) = abs(X_planned(2,jj) - y_temp_final);
                sum_traj_dy_dist=sum_traj_dy_dist+traj_dy_dist(jj);
            end
            if sum_traj_dy_dist < 3.2%Omid: here we want to update  0.7
        disp(Phase);       
        if Phase==1
         if EgoPolicy<0 %then we need to do a lane change 
        scenario_ref='LaneChange';
         else
        scenario_ref='Linear';
         end
        ref_traj = reftraj_gen(2*T, dt, X_planned(2,i-1),20, scenario_ref, aref,vref,tf,X_planned(1,i-1),vehicle_type);%we need to update the x0 from 20 to current position of ego vehicle
        ref_traj_fractions((i-1):1.2*NUM_TOTAL,:)=ref_traj(1:1.2*NUM_TOTAL-(i-2),:);

    elseif Phase==2
    if strcmp(scenario_ref,'Linear')
%     disp('nnnm');
    end
    scenario_ref='LaneChange';
    %y_temp_final y_temp_init;
    % check if the trajectory has reached to its temperary destinations      
    if y_temp_final ~=y_final && strcmp(scenario_ref,'LaneChange') %Omid: here we want to update, we don't want to generate and update linear ref trag  
        disp(EgoPolicy)
%         if EgoPolicy<0 %then we need to do a lane change 
%         scenario_ref='LaneChange';
%         else
%         scenario_ref='Linear';
%         end
%     traj_count=traj_count+1;
        ref_traj   = reftraj_gen(2*T, dt, X_planned(2,i-1),20, scenario_ref, aref,vref,tf,X_planned(1,i-1),vehicle_type);%we need to update the x0 from 20 to current position of ego vehicle
        ref_traj_fractions((i-1):1.2*NUM_TOTAL,:)=ref_traj(1:1.2*NUM_TOTAL-(i-2),:);
    end
    
       
    
        end
            end
%     if ref_traj(1,4) ~= vref% here we want to update ref traj if reftraj(4)~= updated vref which may be equal to target velocity if we want to do lane keeping once we are in tmpoary destinations 
% %              disp(ref_traj(1,4))
%         %vref i constantly updating in DYNCST
%              ref_traj = reftraj_gen(2*T, dt, X_planned(2,i-1),20, scenario_ref, aref,vref,tf,X_planned(1,i-1),vehicle_type);%we need to update the x0 from 20 to current position of ego vehicle
%               ref_traj_fractions((i-1):1.2*NUM_TOTAL,:)=ref_traj(1:1.2*NUM_TOTAL-(i-2),:);
% 
%     end
    end
     
    
    
    tic
    iter = 0;...    iteration counter
    diverge = 0;... levenburg Marquet utility variable
    
    % ==== prediction phase ====
    if use_prediction == 1
        % short term reachability analysis bounding box is given every xxx sec
        % long term prediction is 1 second apart
        for k = 1:length(OBS)
            
            [x_short_term, y_short_term] = reach(tgt_reach_traj(:, i, k), 'linear');... reachability of the target vehicle
            if i == 1
                x = tgt_reach_traj(:,1,k);%added by Omid: it was previously tgt_reach_traj(:,1) and ego wasn't able to see the second target vehicle 
                x_long_term = x*ones(1,NUM_CTRL+1);
                % linear update
                for j = 2:(NUM_CTRL + 1)
                   x_long_term(1,j) = x_long_term(1,j-1) + dt*x_long_term(4,j)*cos(x_long_term(5,j));%check Omid
                   x_long_term(2,j) = x_long_term(2,j-1) + dt*x_long_term(4,j)*sin(x_long_term(5,j));%check Omid
                end 
            elseif i == 2
                x_old = tgt_reach_traj(:,1,k);
                x = tgt_reach_traj(:,2,k);
                ahat_old = diag(ones(1, 3));
                H = 1;
                VAR_old = 0.001*ones(3, 1);%0.0001
                BOUND = ones(3, 1);
                [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H, VAR_old, 40);
            else
                x_old = tgt_reach_traj(:, i-1,k);
                x = tgt_reach_traj(:, i,k);
                ahat_old = ahat;
                VAR_old = VAR;
                H_old = H;   
                % ahat
                [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H, VAR_old, 40);
            end
            x_short_term(end,:) = [];
            y_short_term(end,:) = [];

            x_short_term = reshape(x_short_term, 1,[], 5);
            y_short_term = reshape(y_short_term, 1,[], 5);

            % start from the bottom left and goes counter clockwise
            bounding_box(:,:,:,k) = [x_short_term; y_short_term];

            % inflate the point mass into a vehicle geometry
            bounding_box(:,:,:,k) = bounding_box(:,:,:,k) + ...
                                    (diag([param.len/2;param.wid/2])*...
                                    [-1 1 1 -1;-1 -1 1 1]);

            % record predictions and reachabilities for visualization
            obs_short(:,:,:,i,k) = bounding_box(:,:,:,k);
            if size(x_long_term, 1) == 5%Omid: we only need x,y and tetha for x_long_term
                x_long_term(3,:) = [];
                x_long_term(3,:) = [];
            end
            obs_long(:,:,i,k) = x_long_term;

            % ==== update the target obstacle information ====
            if use_reach == 1
                OBS(k) = OBS(k).load_traj_predicted(x_long_term, bounding_box(:,:,:,k), false);
            else
                OBS(k) = OBS(k).load_traj_predicted(x_long_term, [], false);
            end
        end
    end

%     if (X(1,end) > 32.01)
%     disp(X(1));
%     end

    % ==== CiLQR phase ====
    while iter < maxIter
        %==== STEP 0: initialize derivatives: f_x, f_u, l_x, l_u, l_xx, l_ux, l_uu
            % A_b:  first derivative of A with respect to b
            % A_bc: second derivative of A with respect to b and c
            % A_bb: second derivative of A with respect to b
            % f: vehicle dynamics
            % c: running costs at each timestep
            
        %==== STEP 1: differentiate dynamics and cost along new trajectory
        [cx,cu,cxx,cux,cuu,fx,fu] = DYNCST(X, U, ref_traj, OBS, use_prediction, stop_sign, use_adaptive);
        % end point evaluation, only x no input u
        % end point evaluation of l_x, l_u

        %==== STEP 2: backward pass, compute optimal control law and cost-to-go
        [k, K] = backward_pass(cx,cu,cxx,cux,cuu,fx,fu,lambda);

        %==== STEP 3: line-search to find new control sequence, trajectory, cost
        [X_new, U_new] = forward_pass(U, X, k, K, X_start, dt, alpha);
        
        %==== STEP 4: check new solution for cost
        % see if adapt new solution
        cost_new = calc_cost(X_new, U_new, ref_traj, OBS, use_prediction, stop_sign);
        cost     = calc_cost(X, U, ref_traj, OBS, use_prediction, stop_sign);
        iter     = iter + 1;
        if cost_new < cost
            X = X_new;
            lambda = lambda / lambdaFactor;
            if (abs(cost - cost_new)/cost_new < tolFun)
                diverge = 0;
                break;
            end
        else
            lambda = lambda * lambdaFactor;
            if lambda > lambdaMax
                diverge = 1;
                break;
            end
        end
    end
    
    X_planned(:,i) = X_new(:,1); %actual movement
    U_planned(:,i) = U_new(:,1);
    X_start = X_new(:,2); %why not X_new(:, 1)? %answer: Omid: because we shift trajectory horizon with a dt for each iteration, a new 4s will be checked which is only a dt in front
    U(:,1:NUM_CTRL - 1) = U_new(:,2:end);
    U(:,NUM_CTRL) = zeros(U_DIM, 1); %next nominal control sequence
    [X, ~] = forward_pass(U, [], [], [], X_start, dt, []); %next nominal state sequence
    
    % update the obstacle
    for j = 1:length(OBS)
        if (strcmp(scenario, 'D1') || strcmp(scenario, 'B2') || strcmp(scenario, 'S1-1') || strcmp(scenario, 'S1-2') || strcmp(scenario, 'S1-3') || strcmp(scenario, 'S2'))
            obs_traj(:,i,j) = OBS(j).traj(:,1);
            % trajectory loading update
            OBS(j) = OBS(j).load_traj(cut_in_traj(:,i:i+NUM_CTRL,j), [], false);
            
        else
            % sliding window update based on control
            obs_traj(:,i,j) = OBS(j).traj(:,1);
%             OBS(j) = OBS(j).load_traj(cut_in_traj(:,i:i+NUM_CTRL), [], false);
            OBS(j) = OBS(j).next_window(zeros(U_DIM, NUM_CTRL), false,[]);% using only this will not update the ref traj for obstacles  
            
        end
    end
    loop_time(i) = toc;
%     if (diverge) ... causing weird stuff happeneing here
%         fprintf("unable to converge during inner loop \n");
%         break; 
%     else
%         X_planned(:,i) = X_new(:,1);
%         U_planned(:,i) = U_new(:,1);
%         X_start = X_new(:,2);
%         U(:,1:NUM_CTRL - 1) = U_new(:,2:end);
%         U(:,NUM_CTRL) = zeros(U_DIM, 1);
%         [X, ~] = forward_pass(U, [], [], [], X_start, dt, []);
%     end
    fprintf("i = %d, Converged at %d iteration, cost diff is %4.5f\n", i, ...
        iter, abs(cost_new - cost)/cost);
    % record the planned trajectory within the planning horizon for
    % visualization
    X_local(:,:,i) = X_new;
end

%% visualization
% generate obstacle ellipse
ellipse_ang = -pi:0.05:pi;

rMAT = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];

% movie_time = 0:dt:NUM_TOTAL;
% movie_time = 0:dt:T-T_horizon;
movie_time = 0:dt:T-dt;

numberOfFrames = length(movie_time);

% declare storage vectors for plotting
inflated_car_state = inflate(X_planned);
% car coordinates, one entire row for a rectangle
x = zeros(numberOfFrames, 5);
y = zeros(numberOfFrames, 5);
% obstacle coordinates
x_obs = zeros(numberOfFrames + 1, 5, length(OBS));
y_obs = zeros(numberOfFrames + 1, 5, length(OBS));
for i = 1 : numberOfFrames
%     x(i,:) = inflated_car_state(1,:,i);
%     y(i,:) = inflated_car_state(2,:,i);
    % lower left point
    point_ll = [X_planned(1,i) - 2.5; X_planned(2,i) - 1] - X_planned(1:2,i);
    % lower right point
    point_lr = [X_planned(1,i) + 2.5; X_planned(2,i) - 1] - X_planned(1:2,i);
    % top right point
    point_tr = [X_planned(1,i) + 2.5; X_planned(2,i) + 1] - X_planned(1:2,i);
    % top left point
    point_tl = [X_planned(1,i) - 2.5; X_planned(2,i) + 1] - X_planned(1:2,i);
    % draw car
    point_car = [point_ll, point_lr, point_tr, point_tl, point_ll];
    
    point_rotated = rMAT(X_planned(5,i))*point_car;
    
    point_translated = point_rotated + [X_planned(1:2,i), X_planned(1:2,i), ...
        X_planned(1:2,i), X_planned(1:2,i), X_planned(1:2,i)];
    
    % write to memeory for the ego vehicle
    x(i,:) = point_translated(1,:);
    y(i,:) = point_translated(2,:);
    
    % repeat procedure for target obstacle
    for j = 1:length(OBS)
        % calculate obstacle(i) rectangle corner coordinates
        obs_ll = [obs_traj(1,i,j) - OBS(j).len/2; ...
        obs_traj(2,i,j) - OBS(j).wid/2] - obs_traj(1:2,i,j);
        obs_lr = obs_ll + [OBS(j).len; 0];
        obs_tr = obs_ll + [OBS(j).len; OBS(j).wid];
        obs_tl = obs_ll + [0; OBS(j).wid];
        obs_pt = [obs_ll, obs_lr, obs_tr, obs_tl, obs_ll];
        obs_rot = rMAT(obs_traj(5,i,j)) * obs_pt;
        obs_trans = obs_rot + [obs_traj(1:2,i,j),obs_traj(1:2,i,j),...
            obs_traj(1:2,i,j),obs_traj(1:2,i,j),obs_traj(1:2,i,j)];
        % write to memory
        x_obs(i,:,j) = obs_trans(1,:);
        y_obs(i,:,j) = obs_trans(2,:);
    end    
end

% summarizes car vertices
car_vert = [reshape(x(:,1:4), 1,4,[]); reshape(y(:,1:4), 1,4,[])];
%% GIF
plot_gif = 1;
pOBS = zeros(5, length(OBS));
% % [img, map, alphachannel] = imread('egoV.png');
% % image(img, 'alphadata', alphachannel);
% [egoimage, map, egoimage_alpha]  = imread('egoV.png');

% egoimage = imresize(egoimage, 0.1);
if plot_gif == 1
    figure()
    delay = 0.1;
    for i = 1 : numberOfFrames
%         P1=0;P2=0;P3=0;P4=0;P5=0;P6=0;
        % draw car
        r = plot(x(i,:),y(i,:),'k');
        hold on
        % draw planned trajectory and the reference trajectory
        
        plot(X_planned(1,:), X_planned(2,:),'b');
        hold on
%         if ref_traj_fractions
        plot(ref_traj_fractions(:,1),ref_traj_fractions(:,2),'--g');  
        hold on
%         else
%             plot(ref_traj(:,1),ref_traj(:,2),'--g');  
%         hold on
%         end
%         image(egoimage, 'alphadata', egoimage_alpha);
%         hold on

        % draw obstacle
        for j = 1:length(OBS)
            obs_rec(j) = plot(x_obs(i,:,j), y_obs(i,:,j),'m','LineWidth', 1);
            hold on
            if cut_in_traj(1,i,j)>0 && cut_in_traj(1,i,j)<NUM_TOTAL+50
            if ((cut_in_traj(2,i,j)==(road_up_lim-Lane_size/2.))) 

            pOBS(1,j)=text(cut_in_traj(1,i,j)-3,cut_in_traj(2,i,j)+12, 'V=' );
            hold on
            pOBS(2,j)=text(cut_in_traj(1,i,j)+3,cut_in_traj(2,i,j)+12, num2str(cut_in_traj(4,i,j)) );
            hold on
            pOBS(3,j)=text(cut_in_traj(1,i,j)-3,cut_in_traj(2,i,j)+8, '\DeltaX(1)=' );
            hold on
            pOBS(4,j)=text(cut_in_traj(1,i,j)+10,cut_in_traj(2,i,j)+8, num2str(cut_in_traj(1,i,j)-X_planned(1,i),4) );
            hold on
            pOBS(5,j)=text(cut_in_traj(1,i,j)-2,cut_in_traj(2,i,j)+5, '\downarrow' );
            hold on
            else
            pOBS(1,j)=text(cut_in_traj(1,i,j)-3,cut_in_traj(2,i,j)-12, 'V=' );
            hold on
            pOBS(2,j)=text(cut_in_traj(1,i,j)+3,cut_in_traj(2,i,j)-12, num2str(cut_in_traj(4,i,j)) );
            hold on
            pOBS(3,j)=text(cut_in_traj(1,i,j)-3,cut_in_traj(2,i,j)-8, '\DeltaX(1)=' );
            hold on
            pOBS(4,j)=text(cut_in_traj(1,i,j)+10,cut_in_traj(2,i,j)-8, num2str(cut_in_traj(1,i,j)-X_planned(1,i),4) );
            hold on
            pOBS(5,j)=text(cut_in_traj(1,i,j)-2,cut_in_traj(2,i,j)-5, '\uparrow' );
            hold on
            end
            end
            
        end
        if length(OBS)==0
            obs_rec=0;
        end

    %     obs1_ellip_x = (obs1_axis(1,i))*cos(ellipse_ang);
    %     obs1_ellip_y = (obs1_axis(2,i))*sin(ellipse_ang);
    %     obs_ellip_rot =[obs1_traj(1,i);obs1_traj(2,i)] + rMAT(obs1_traj(4,i)) * [obs1_ellip_x; obs1_ellip_y];
    %     
    %     o = plot(obs_ellip_rot(1,:), obs_ellip_rot(2,:),'g');
    %     hold on
        p = plot(X_local(1,1:5,i), X_local(2,1:5,i), 'b--*');
        hold on

        % environment visualizations
%         plot(side_lane(:,1),side_lane(:,2),'--r');  
%         hold on
%         plot(top_lane(:,1),top_lane(:,2),'--r');
%         hold on
        plot(left_road(:,1),left_road(:,2),'r');  
        hold on
        plot(right_road(:,1),right_road(:,2),'r');  
        hold on
        if road_up_lim >Lane_size%we only want to draw these lines for highway more than 2 lanes 
            plot(left_road_half(:,1),left_road_half(:,2),'--r');  
        hold on
        plot(right_road_half(:,1),right_road_half(:,2),'--r');  
        hold on
        end
%         plot(mid_road(:,1),mid_road(:,2),'r');  
        plot(mid_road(:,1),mid_road(:,2),'--r'); %added by Omid for one sided 4 lanes highway
        hold on
        axis equal

        if use_prediction == 1
            % plot bounding boxes
            for hh = 1:length(OBS)% added by Omid
            for j = 1:5
                bb(j,hh) = plot(obs_short(1,:,j,i,hh), obs_short(2,:,j,i,hh), 'g');
            end
            end
            % plot long term prediction 
    %         lp = plot(obs_long(1,:,i), obs_long(2,:,i), 'c*');
        end

        xlim([0 (NUM_TOTAL+50)]); ylim([(road_low_lim-15) (road_up_lim+15)]);
        xlabel('x(m)', 'FontSize', 25); ylabel('y(m)', 'FontSize', 25);
    %     legend('vehicle','planned','ref','obstacle','planning horizon','Location','best');

        % set plot position
        x0 = 0;
        y0 = 0;
%         width = 960;
%         height = 540;
        width = 1920;
        height = 1080;
        set(gcf,'position',[x0,y0,width,height]);

        drawnow;

        thisFrame = getframe(gca);
        im = frame2im(thisFrame);
          [imind,cm] = rgb2ind(im,256);
          if i == 1
              imwrite(imind,cm,'test.gif','gif', 'DelayTime', delay, 'Loopcount',inf);
          else
              imwrite(imind,cm,'test.gif','gif','WriteMode','append', 'DelayTime', delay);
          end

        delete(p);
        
        for j = 1:length(OBS)
            if cut_in_traj(1,i,j)>0 && cut_in_traj(1,i,j)<NUM_TOTAL+50
            delete(pOBS(1,j));delete(pOBS(2,j));delete(pOBS(3,j));delete(pOBS(4,j));delete(pOBS(5,j));
            end
        end
    %     delete(o)
        delete(r)
        if length(OBS)~=0

           delete(obs_rec)
           if use_prediction == 1
            delete(bb)
    %         delete(lp)
           end
        end
        
    end
end


%% paper plotting
figure()

subplot(4, 1, 1);
interval = 10;
i0 = 1;
it = NUM_TOTAL;
% environment visualizations
plot(side_lane(:,1),side_lane(:,2)+6,'--r');  
hold on
plot(side_lane(:,1),side_lane(:,2),'-.k');  
hold on
plot(left_road(:,1),left_road(:,2),'k');  
hold on
plot(right_road(:,1),right_road(:,2),'k');  
hold on
plot(mid_road(:,1),mid_road(:,2),'k');  
hold on

% draw middle part
for i = i0:interval:it
    plot_alpha = (i-i0)/(it-i0) * 0.7;
    % draw car
    h(i) = fill(x(i,:),y(i,:),'blue');
    h(i).FaceAlpha = plot_alpha;
    hold on;
    % draw obstacle
    for j = 1:length(OBS)
        obs_rec(i,j) = fill(x_obs(i,:,j), y_obs(i,:,j),'red');
        obs_rec(i,j).FaceAlpha = plot_alpha;
        hold on
    end
    
%     if i >= 110 && i <= 120
%         % draw prediction
%         plot(X_local(1,1:5,i), X_local(2,1:5,i), 'b--*', 'LineWidth', 0.1);
%         hold on
% 
%         % draw reachability
%         for j = 1:5
%             plot([obs_short(1,:,j,i) obs_short(1,1,j,i)], [obs_short(2,:,j,i),obs_short(2,1,j,i)], 'g','LineWidth', 2);
%         end
%     end
end

% draw past trajectory
plot(X_planned(1,:), X_planned(2,:), 'b');
hold on
% draw target trajectory
for k = 1:length(OBS)
    plot(obs_traj(1,:,k), obs_traj(2,:,k), 'r');
    hold on
end
plot(ref_traj_fractions(:,1),ref_traj_fractions(:,2),'-.k');  
hold on

axis equal
xlim([0 NUM_TOTAL]); % B2
ylim([-8 8]);
% % set plot position
% x0 = 0;
% y0 = 0;
% width = 1920;
% height = 1080;
% set(gcf,'position',[x0,y0,width,height]);

subplot(4, 1, 2);
interval = 10;
i0 = 1;
it = NUM_TOTAL;
% environment visualizations
plot(side_lane(:,1),side_lane(:,2)+6,'--r');  
hold on
plot(side_lane(:,1),side_lane(:,2),'-.k');  
hold on
plot(left_road(:,1),left_road(:,2),'k');  
hold on
plot(right_road(:,1),right_road(:,2),'k');  
hold on
plot(mid_road(:,1),mid_road(:,2),'k');  
hold on


% draw target trajectory
for k = 1:length(OBS)
    plot(obs_traj(1,:,k), obs_traj(2,:,k), 'r');
    hold on
end
plot(ref_traj_fractions(:,1),ref_traj_fractions(:,2),'-.k');  
hold on

axis equal
xlim([0 NUM_TOTAL]); % B2
ylim([-8 8]);

subplot(4, 1, 3);
yyaxis left
if length(OBS)~=0
plot(obs_traj(4,:,1), 'm', 'LineWidth', 2);
end
hold on
plot(X_planned(4, :), '-b', 'LineWidth', 2);
ylabel('Velocity (m/s)', 'FontSize', 25);
xlabel('longitudinal displacement (m)', 'FontSize', 25);
ylim([-1 18])

yyaxis right
plot(X_planned(3,:), 'LineWidth', 2);
ylabel('Acceleration (m/s^2)', 'FontSize', 25);
xlim([0 NUM_TOTAL]);
ylim([-5 10])
grid on
legend('target velocity', 'ego velocity', 'ego acceleration', 'FontSize',25, 'Location', 'northwest');

% acceleration and jerk profile (Omid)
subplot(4, 1, 4);
yyaxis left
plot(X_planned(3, :), '-b', 'LineWidth', 2);
ylabel('Acceleration (m/s^2)', 'FontSize', 25);
xlabel('longitudinal displacement (m)', 'FontSize', 25);
ylim([-5 10])

yyaxis right
plot(U_planned(1,:), 'LineWidth', 2);
ylabel('Jerk (m/s^3)', 'FontSize', 25);
xlim([0 NUM_TOTAL]);
ylim([-3 3])
grid on
legend('ego acceleration', 'ego jerk', 'FontSize',25, 'Location', 'northwest');

% set subplot position
set(subplot(4,1,1), 'Position', [0.075, 0.78, 0.85, 0.35], 'FontSize', 25)
set(subplot(4,1,2), 'Position', [0.075, 0.68, 0.85, 0.35], 'FontSize', 25)
set(subplot(4,1,3), 'Position', [0.075, 0.48, 0.85, 0.30], 'FontSize', 25)
set(subplot(4,1,4), 'Position', [0.075, 0.08, 0.85, 0.30], 'FontSize', 25)

% set plot position
x0 = 0;
y0 = 0;
width = 1920;
height = 1780;
set(gcf,'position',[x0,y0,width,height]);

if (use_reach == 1)
    saveas(gcf, [scenario, '_reach.png']);
else
    saveas(gcf, [scenario, '_no_reach.png']);
end
%% contol plotting
figure()
yyaxis left
plot(U_planned(1,:));
ylabel('Jerk');

yyaxis right
plot(U_planned(2,:)*180/pi);
ylabel('Steering Angle');
xlabel('longitudinal distance (m)');
title('Control Effort vs Distance Plot');
width = 960;
height = 540;
set(gcf,'position',[x0,y0,width,height]);

%% velocity plot
figure()
plot(X_planned(4,:));
xlabel('longitudinal distance (m)');
ylabel('velocity (m/s)');
title('Velocity vs Distance Plot');
width = 960;
height = 540;
set(gcf,'position',[x0,y0,width,height]);
