function [cx,cu,cxx,cux,cuu,fx,fu] = DYNCST(X, U, ref_traj, obstacle, ...
                                        use_prediction, stopping, adaptive)
% DYNCST: calculate all necessary first and second derivatives
%
% all derivatives are matrices
%
% input
% ======
% X:        state, size(X) = XDIM * (N+1)
% U:        control sequence, size(U) = UDIM * (N)
% ref_traj: reference trajectory
% obstacle: obstacle class
% use_prediction: 0 to use deterministic model, and 1 to use prediction
%
% output
% =====
% cx:       first derivative of cost fcn w.r.t state X
% cu:       first derivative of cost fcn w.r.t control u
% cxx:      second derivative of cost fcn w.r.t state X
% cux:      second derivative of cost fcn w.r.t control u and state X
% cuu:      second derivative of cost fcn w.r.t control u
% fx:       first derivative of car dynamics w.r.t state X
% fu:       first derivative of car dynamics w.r.t state u

% load global parameters
global X_DIM U_DIM NUM_CTRL L dt
global a_dot_max a_dot_min v_dot_max v_dot_min delta_max delta_min
% load cost_weights
global w_ref w_vel w_jerk w_acc w_del w_end_ref w_end_acc w_end_vel
global q1_back q2_back
global q1_front q2_front
global q1_stop q2_stop
global q1_road q2_road
global q1_CenterLane q2_CenterLane %added by Omid
global v_min q1_min_vel q2_min_vel

global q1_jerk q2_jerk q1_acc q2_acc q1_del q2_del
w_ref = zeros(NUM_CTRL,1);
w_vel = zeros(NUM_CTRL,1);
w_acc = zeros(NUM_CTRL,1);
w_jerk = zeros(NUM_CTRL,1);
w_del = zeros(NUM_CTRL,1);
q2_jerk = zeros(NUM_CTRL,1);q1_jerk= zeros(NUM_CTRL,1);
q2_del = zeros(NUM_CTRL,1); q1_del= zeros(NUM_CTRL,1) ;
q2_acc = zeros(NUM_CTRL,1);q1_acc= zeros(NUM_CTRL,1);
q2_back = zeros(NUM_CTRL,1); q1_back= zeros(NUM_CTRL,1); 
q2_front = zeros(NUM_CTRL,1); q1_front= zeros(NUM_CTRL,1);
q2_road = zeros(NUM_CTRL,1); q1_road= zeros(NUM_CTRL,1);

if (adaptive == 0)
    global w_ref w_vel
else
    global a_vel b_vel a_ref b_ref a_acc b_acc a_road b_road
end

global param
% stop sign location
global x_stop

% road upper and lower limits
global road_up_lim road_low_lim Lane_size
global vref

%% vehicle kinematcs differential terms
fx = zeros(X_DIM, X_DIM, NUM_CTRL);
fu = zeros(X_DIM, U_DIM, NUM_CTRL);

for i = 1:1:NUM_CTRL
    acc     = X(3,i);
    v       = X(4,i);
    theta   = X(5,i);
    jerk     = U(1,i);
    delta   = U(2,i);
    
    k       = tan(delta)/L;...              curvature
    l       = v*dt + 0.5*acc*dt^2+(1/6.)*jerk*dt^3;...       distance traveled % jerk added by Omid
    phi     = theta + k*l;

    % df_dx
    if (k == 0)
        fx(:,:,i) = eye(X_DIM) + ...
                    [0, 0, 0.5*(dt^2)*cos(phi), dt*cos(phi), -l*sin(theta);
                     0, 0, 0.5*(dt^2)*sin(phi), dt*sin(phi), l*cos(theta);
                     0, 0, 0, 0,           0;
                     0, 0,dt, 0,           0;
                     0, 0,0.5*(dt^2)*k, dt*k,        0];
    else
        fx(:,:,i) = eye(X_DIM) + ...
                    [0, 0, 0.5*(dt^2)*cos(phi), dt*cos(phi), 1/k*(cos(phi) - cos(theta));
                     0, 0, 0.5*(dt^2)*sin(phi), dt*sin(phi), 1/k*(sin(phi) - sin(theta));
                     0, 0, 0, 0,           0;
                     0, 0, dt, 0,          0;
                     0, 0, 0.5*(dt^2)*k, dt*k,        0];%updated for jerk min x=[px,py,a,v,tetha], Omid
    end
    
    
    % df_du
    dk = (sec(delta))^2/L;
    if (k == 0)
        fu(:,:,i) = [cos(phi)*(1/6.)*dt^3,  0;
                     sin(phi)*(1/6.)*dt^3,  0;
                     dt,                 0;
                     0,                 0;
                     0,                  l * dk];
    else
        fu(:,:,i) = [cos(phi)*(1/6.)*dt^3, (l/k*cos(phi) - 1/k^2*(sin(phi)-sin(theta)))*dk;
                     sin(phi)*(1/6.)*dt^3, (l/k*sin(phi) + 1/k^2*(cos(phi)-cos(theta)))*dk;
                     dt,                 0;
                     0,                 0;
                     k*(1/6.)*dt^3,         l * dk];
    end


end

%% cost function and constraints
cx  = zeros(X_DIM, 1, NUM_CTRL + 1);
cxx = zeros(X_DIM, X_DIM, NUM_CTRL + 1);
cu  = zeros(U_DIM, 1, NUM_CTRL);
cuu = zeros(U_DIM, U_DIM, NUM_CTRL);
cux = zeros(U_DIM, X_DIM, NUM_CTRL + 1);...     du then dx

for i = 1:1:NUM_CTRL
    Xi = X(:,i);
    Ui = U(:,i);
    % adaptive weight calculation
    if (adaptive == 1)
        for k = 1:length(obstacle)
%         dx_wei = sqrt((Xi(1) - obstacle(1).traj(1,i,k))^2 + (Xi(2) - obstacle(1).traj(2,i,k))^2);
%         w_ref = abs(obstacle(1).traj(3,i))/a_ref / exp(b_ref * dx_wei);
%         w_vel = a_vel/abs(obstacle(1).traj(3,i)) * exp(b_vel * dx_wei);
%         disp(length(obstacle))
%         disp(obstacle(k).traj(1,i))
        % adaptive weight calculations
         dx_wei_tags1(k) = sqrt((Xi(1) - obstacle(k).traj(1,i))^2 + (Xi(2) - obstacle(k).traj(2,i))^2);% (k) is added by Omid to see all the targets
        
        if (CenterLaneY_detector(Xi(2))==CenterLaneY_detector(obstacle(k).traj(2,i)))
            if(Xi(1) < obstacle(k).traj(1,i))
            dx_wei_tags2(k) = sqrt((Xi(1) - obstacle(k).traj(1,i))^2 + (Xi(2) - obstacle(k).traj(2,i))^2);% (k) is added by Omid to see all the targets
            end
        else    
            dx_wei_tags2(k)=100;
        end
        
        end
        
        [dx_wei,index]=min(dx_wei_tags1);% addaptive weight func modified by Omid
%         w_ref(i) = obstacle(index).traj(4,i) / (a_ref* exp(b_ref * dx_wei));
        w_ref(i) = a_ref.*(obstacle(index).traj(4,i)/vref)/exp(b_ref * dx_wei.^0.25);
        w_acc(i) = (obstacle(index).traj(4,i)/vref)/ exp(b_acc * dx_wei.^0.25);
        w_jerk(i) = 0.05*(obstacle(index).traj(4,i)/vref)/ exp(b_acc * dx_wei.^0.25);
        w_del(i) = (obstacle(index).traj(4,i)/vref)/ exp(b_acc * dx_wei.^0.25);
        q1_jerk(i) = 0.05.*(obstacle(index).traj(4,i)/vref)/ exp(b_acc * dx_wei.^0.25);
        q2_jerk(i) = q1_jerk(i) ;
        q1_del(i) = (obstacle(index).traj(4,i)/vref)/ exp(b_acc * dx_wei.^0.25);
        q2_del(i) = q1_del(i) ;
        q1_acc(i) = (obstacle(index).traj(4,i)/vref)/ exp(b_acc * dx_wei.^0.25);
        q2_acc(i) = q1_acc(i) ;
        q1_back(i) = exp(b_acc * dx_wei.^0.25)/(obstacle(index).traj(4,i)/vref) ;
        q2_back(i) = q1_back(i); q1_front(i) = q1_back(i); q2_front(i) = q1_back(i);
        q1_road(i) = a_road*q1_back(i);
        q2_road(i) = q1_road(i);
         
%         w_acc(i) =w_ref(i)/2. ;
        
        [dx_wei,index]=min(dx_wei_tags1);% addaptive weight func modified by Omid
%         w_vel(i) =  (obstacle(index).traj(4,i)/vref)/exp(b_vel * dx_wei.^0.25);
        w_vel(i) =  a_vel*exp(b_vel * dx_wei.^0.25)/(obstacle(index).traj(4,i)/vref) ;
%         w_vel(i) =  1/( exp(b_vel * dx_wei.^0.25)*(obstacle(index).traj(4,i)/vref));
%         w_vel(i+1) = a_vel / obstacle(index).traj(4,i) / exp(b_vel * dx_wei);
%         w_vel(i) =w_ref(i)/6. ;
%         w_acc(i) =0. ;
%         xxx = [w_ref,'     ',w_vel];
% disp(w_ref)
% disp(w_vel)
% disp(index)
% disp('     ')

%         disp(num2str(w_ref),'   ',num2str());
        
    end
    
    % reference trajectory
    [~, index] = min(sum((ref_traj' - Xi).^2));
    Xref = transpose(ref_traj(index,:));
    cx(:,:,i) = cx(:,:,i) + w_ref(i) * diag([2 2 0 0 0]) * (Xi - Xref);...   4x1
    cxx(:,:,i) = cxx(:,:,i) + w_ref(i) * diag([2 2 0 0 0]);...               4x4
    
    % acceleration
    cx(:,:,i) = cx(:,:,i) + w_acc(i) * diag([0 0 2 0 0]) * (Xi - Xref);...   4x1
    cxx(:,:,i) = cxx(:,:,i) + w_acc(i)* diag([0 0 2 0 0]);...               4x4
    
    % velocity
    cx(:,:,i) = cx(:,:,i) + w_vel(i) * diag([0 0 0 2 0]) * (Xi - Xref);...   4x1
    cxx(:,:,i) = cxx(:,:,i) + w_vel(i) * diag([0 0 0 2 0]);...               4x4
    
    % control
    cu(:,:,i) = cu(:,:,i) + diag([w_jerk(i) w_del(i)]) * diag([2 2]) * (Ui);...              2x1
    cuu(:,:,i) = cuu(:,:,i) + diag([w_jerk(i) w_del(i)]) * diag([2 2]);...                   2x2
    
    % obstacle term
    for j = 1:length(obstacle)
        % only calculates one step derivative cost instead of a sequence
        [bx, bxx] = calc_bx(X, obstacle(j), i, ...
            q1_back(i), q2_back(i), q1_front(i), q2_front(i), use_prediction);
        cx(:,:,i) = cx(:,:,i) + bx;...                                  4x1
        cxx(:,:,i) = cxx(:,:,i) + bxx;...                               4x4
    end

    % ==================== road limits ====================
    % upper bound
    % road upper and lower limits
    g   = Xi(2) - (road_up_lim-param.wid/2.);
    dg 	= [0;1;0;0;0];
    db  = q1_road(i) * q2_road(i) * exp(q2_road(i) * g) * (dg);              % scalar * (4x1) 
    ddb = q1_road(i) * q2_road(i) ^2 * exp(q2_road(i) * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % lower bound
    g   = (road_low_lim+param.wid/2.) - Xi(2);
    dg 	= [0;1;0;0;0];
    db  = q1_road(i) * q2_road(i) * exp(q2_road(i) * g) * (dg);              % scalar * (4x1) 
    ddb = q1_road(i) * q2_road(i) ^2 * exp(q2_road(i) * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % ====================  penalizing distanc from the lane center (Omid)  ====================
    CenterLaneY = CenterLaneY_detector(Xi(2));
    g   = abs(Xi(2)-CenterLaneY) - Lane_size/2.;
    dg 	= [0;1;0;0;0];
    db  = q1_CenterLane * q2_CenterLane * exp(q2_CenterLane * g) * (dg);              % scalar * (4x1) 
    ddb = q1_CenterLane * q2_CenterLane ^2 * exp(q2_CenterLane * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
     % ====================acceleration limit ====================

    %min velocity constraint
    g   =v_min-Xi(4);
    dg 	= [0;0;0;1;0];
    db  = q1_min_vel * q2_min_vel * exp(q2_min_vel * g) * (dg);              % scalar * (2x1) 
    ddb = q1_min_vel * q2_min_vel ^2 * exp(q2_min_vel * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % acceleration
    % upper bound
    g   = Xi(3) - v_dot_max;
    dg 	= [0;0;1;0;0];
    db  = q1_acc(i) * q2_acc(i) * exp(q2_acc(i) * g) * (dg);              % scalar * (2x1) 
    ddb = q1_acc(i) * q2_acc(i) ^2 * exp(q2_acc(i) * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    
    % lower bound
    g   = v_dot_min - Xi(3);%Omid this must be Xi(3) for acceleration
    dg 	= [0;0;1;0;0];
    db  = q1_acc(i) * q2_acc(i) * exp(q2_acc(i) * g) * (dg);              % scalar * (2x1) 
    ddb = q1_acc(i) * q2_acc(i) ^2 * exp(q2_acc(i) * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4
    % ==================== ctrl limits ====================
    % jerk
    % upper bound
    g   = Ui(1)  - a_dot_max;
    dg 	= [1;0];
    db  = q1_jerk(i) * q2_jerk(i) * exp(q2_jerk(i) * g) * (dg);              % scalar * (2x1) 
    ddb = q1_jerk(i) * q2_jerk(i)^2 * exp(q2_jerk(i) * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % lower bound
    g   = a_dot_min - Ui(1) ;%Omid this must be Xi(3) for acceleration
    dg 	= [1;0];
    db  = q1_jerk(i) * q2_jerk(i) * exp(q2_jerk(i) * g) * (dg);              % scalar * (2x1) 
    ddb = q1_jerk(i) * q2_jerk(i) ^2 * exp(q2_jerk(i) * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % steering
    % upper bound
    g   = Ui(2) - delta_max;
    dg 	= [0;1];
    db  = q1_del(i) * q2_del(i) * exp(q2_del(i) * g) * (dg);              % scalar * (2x1) 
    ddb = q1_del(i) * q2_del(i)^2 * exp(q2_del(i) * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % lower bound 
    g   = delta_min - Ui(2);
    dg 	= [0;1];
    db  = q1_del(i)* q2_del(i) * exp(q2_del(i) * g) * (dg);              % scalar * (2x1) 
    ddb = q1_del(i) * q2_del(i) ^2 * exp(q2_del(i) * g) * (dg) * (dg');   % scalar * (2x1) * (1x2)

    cu(:,:,i)   = cu(:,:,i) + db;...                                 2x1
    cuu(:,:,i)  = cuu(:,:,i) + ddb;...                               2x2
    
    % ==================== stop sign ====================
    if (stopping == 1)
        % stop sign at 75 meters
        g   = Xi(1) - x_stop;
        dg 	= [1;0;0;0;0];
        db  = q1_stop * q2_stop * exp(q2_stop * g) * (dg);              % scalar * (4x1) 
        ddb = q1_stop * q2_stop ^2 * exp(q2_stop * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)
        
        cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
        cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4


%         cx(:,:,i)   = cx(:,:,i) + 2 * diag([w_end_ref 0 w_end_vel 0]) * ...
%             [Xi(1) - x_stop; 0; Xi(3); 0];...               4x1
%         cxx(:,:,i)  = cxx(:,:,i) + 2 * ...
%             diag([w_end_ref 0 w_end_vel 0]);...  	4x4
    end
end

%% end state
i  = NUM_CTRL + 1;
Xi = X(:,i);

% reference trajectory and velocity cost
[~, index] = min(sum((ref_traj' - Xi).^2));
Xref = transpose(ref_traj(index,:));
cx(:,:,i) = cx(:,:,i) + 2 * diag([w_end_ref w_end_ref w_end_acc w_end_vel 0]) *  (Xi - Xref);...	4x1
cxx(:,:,i) = cxx(:,:,i) + 2 * diag([w_end_ref w_end_ref w_end_acc w_end_vel 0]);...              	4x4

% added by Omid to get w_vel w_ref fo i=41
%  if (adaptive == 1)
%         for k = 1:length(obstacle)
% 
%         % adaptive weight calculations
%         dx_wei_tags(k) = sqrt((Xi(1) - obstacle(k).traj(1,i))^2 + (Xi(2) - obstacle(k).traj(2,i))^2);% (k) is added by Omid to see all the targets
% 
%         end
%         [dx_wei,index]=min(dx_wei_tags);% addaptive weight func modified by Omid
%         w_ref(i) = obstacle(index).traj(4,i) / a_ref * exp(b_ref * dx_wei);
%         w_vel(i) = a_vel / obstacle(index).traj(4,i) / exp(b_vel * dx_wei);
%     end


% road limits
% upper bound
g   = Xi(2) - (road_up_lim-param.wid/2.);
dg 	= [0;1;0;0;0];
db  = q1_road(i-1) * q2_road(i-1) * exp(q2_road(i-1) * g) * (dg);              % scalar * (4x1) 
ddb = q1_road(i-1) * q2_road(i-1) ^2 * exp(q2_road(i-1) * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4

% lower bound
g   = (road_low_lim+param.wid/2.) - Xi(2);
dg 	= [0;1;0;0;0];
db  = q1_road(i-1) * q2_road(i-1) * exp(q2_road(i-1) * g) * (dg);              % scalar * (4x1) 
ddb = q1_road(i-1) * q2_road(i-1) ^2 * exp(q2_road(i-1) * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4

% ==================== penalizing distanc from the lane center (Omid) ====================
    CenterLaneY = CenterLaneY_detector(Xi(2));
    g   = abs(Xi(2)-CenterLaneY) - Lane_size/2.;
    dg 	= [0;1;0;0;0];
    db  = q1_CenterLane * q2_CenterLane * exp(q2_CenterLane * g) * (dg);              % scalar * (4x1) 
    ddb = q1_CenterLane * q2_CenterLane ^2 * exp(q2_CenterLane * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i)   = cx(:,:,i) + db;...                                 4x1
    cxx(:,:,i)  = cxx(:,:,i) + ddb;...                               4x4


% obstacle term
if (~isempty(obstacle))
    for j = 1:length(obstacle)
        % only calculates one step derivative cost instead of a sequence
        [bx, bxx] = calc_bx(X, obstacle(j), i, q1_back(i-1), q2_back(i-1), q1_front(i-1), q2_front(i-1), use_prediction);
        cx(:,:,i) = cx(:,:,i) + bx;...                                      4x1
        cxx(:,:,i) = cxx(:,:,i) + bxx;...                                   4x4
    end
end

% stop sign
if (stopping == 1)
    % stop sign at 75 meters
    g   = Xi(1) - x_stop;
    dg 	= [1;0;0;0;0];
    db  = q1_stop * q2_stop * exp(q2_stop * g) * (dg);              % scalar * (4x1) 
    ddb = q1_stop * q2_stop ^2 * exp(q2_stop * g) * (dg) * (dg');   % scalar * (4x1) * (1x4)

    cx(:,:,i) = cx(:,:,i) + db;...                                  4x1
    cxx(:,:,i) = cxx(:,:,i) + ddb;...                               4x4

%     cx(:,:,i)   = cx(:,:,i) + 2 * diag([w_end_ref 0 w_end_vel 0]) * ...
%         [Xi(1) - x_stop; 0; Xi(3); 0];...               4x1
%     cxx(:,:,i)  = cxx(:,:,i) + 2 * ...
%         diag([w_end_ref 0 w_end_vel 0]);...  	4x4
end

end
