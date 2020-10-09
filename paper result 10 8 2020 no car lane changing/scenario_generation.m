function [obs, cut_in_traj] = scenario_generation(Case_Code)
% this function offers the scenario generation for different cases
% Case_Code refers to:
% Defensive Planning
%   D1 -- one target vehicle cuts in
%   D2 -- one target vehicle cuts in, one target vehicle coming in the 
%           opposite lane (not feasible scenario)
%   D3 -- one target vehicle is our initial ACC target, one target vehicle
%           tries to cut into the gap
% Active Planning
%   A1 -- one front target vehicle travels slowly (overtake)
%   A2 -- one front target vehicle travels close to 
%           a reference velocity (lane keeping)
%   A3 -- one front target vehicle traverls slowly, one target vehicle 
%           incoming in the opposite lane (safe overtake while merging
%           back)
%   A4 -- one front target vehicle to follow and lane changing 
%           to an open lane
%   A5 -- one front target vehicle to follow and lane changing 
%           to the next lane with target vehicle in front
%   A6 -- one front target vehicle to follow and lane changing 
%           to the next lane with target vehicles front and behind
%
% output
% ======
% obs: compiled obstacles into an array with obs(1) being the target
% vehicle to execute moves on
%
% cut_in_traj: more like the idea of full target vehicle nominal trajectory
% it's dimensions will be X_DIM x NUM_TOTAL x n, with n being obstacle
% numbers

    global NUM_CTRL
    global T
    global T_horizon
    global dt
    global X_DIM
    global U_DIM
    global t_switch
    global Lane_size

    if strcmp(Case_Code,'D1')

        obs1 = obstacle([5;-3], [], 0,12, 0, NUM_CTRL, dt, t_switch);
        obs1 = obs1.update_ctrl(zeros(U_DIM, NUM_CTRL));
        % target performing lane change
        % last four inputs: (type, velocity, tf, x0)
        cut_in_traj = (reftraj_gen(T+T_horizon, dt, -3, 'LaneChange', 0,8, 3, 5))'; 
        obs1 = obs1.load_traj(cut_in_traj(:,1:1+NUM_CTRL), [], false);
        % compile into an obstacle 
        obs = [obs1];
    elseif strcmp(Case_Code,'D2')... this scenario does not make sense
%         % obs1
%         obs1 = obstacle([20;-3], [], 10, 0, NUM_CTRL, dt, t_switch);
%         obs1 = obs1.updwate_ctrl(zeros(U_DIM, NUM_CTRL));
%         % target performing lane change
%         cut_in_traj = (reftraj_gen(T+T_horizon, dt, -3, 'LaneChange', 10, 5, 20))'; 
%         obs1 = obs1.load_traj(cut_in_traj(:,1:1+NUM_CTRL), [], false);
%         % obs2
%         obs2 = obstacle([100;3], [], -12, 0, NUM_CTRL, dt, t_switch);
%         obs2 = obs2.update_ctrl(zeros(U_DIM, NUM_CTRL));
%         obs2 = obs2.update_traj(false);
%         % concatenate two obstacles 
%         obs  = [obs1, obs2];
    elseif strcmp(Case_Code,'A1')
        obs_init = [20;-3;8;0];
        obs1 = obstacle(obs_init(1:2), [], obs_init(3), obs_init(4), NUM_CTRL, dt, t_switch);
        obs1 = obs1.update_traj(false);
        % concatenate two obstacles 
        obs = [obs1];
        % build a full target trajectory
        cut_in_traj = obs_init * ones(1, (T+T_horizon)/dt);
        a_max = 6;
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i) = cut_in_traj(1,i-1) + cos(cut_in_traj(4,i-1)) * cut_in_traj(3,i-1) * dt;
            cut_in_traj(2,i) = cut_in_traj(2,i-1) + sin(cut_in_traj(4,i-1)) * cut_in_traj(3,i-1) * dt;
        end
    elseif strcmp(Case_Code,'A2')
        obs_init = [10;-3;12;0];
        obs1 = obstacle(obs_init(1:2), [], obs_init(3), obs_init(4), NUM_CTRL, dt, t_switch);
        obs1 = obs1.update_traj(false);
        % concatenate two obstacles 
        obs = [obs1];
        % build a full target trajectory
        cut_in_traj = obs_init * ones(1, (T+T_horizon)/dt);
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i) = cut_in_traj(1,i-1) + cos(cut_in_traj(4,i-1)) * cut_in_traj(3,i-1) * dt;
            cut_in_traj(2,i) = cut_in_traj(2,i-1) + sin(cut_in_traj(4,i-1)) * cut_in_traj(3,i-1) * dt;
        end
    elseif strcmp(Case_Code,'A3')
        % obstacle1
        o1_init = [20;-3;8;0];
        o1_vert = inflate(o1_init);
        o1      = obstacle(o1_init(1:2), o1_vert, o1_init(3), o1_init(4), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [150; 3; 10; -pi];
        o2_vert = inflate(o2_init);
        o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        obs = [o1, o2];
        
        % build a full target trajectory
        cut_in_traj(:,:,1) = o1_init * ones(1, (T+T_horizon)/dt);
        cut_in_traj(:,:,2) = o2_init * ones(1, (T+T_horizon)/dt);
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i,1) = cut_in_traj(1,i-1,1) + cos(cut_in_traj(4,i-1,1)) * cut_in_traj(3,i-1,1) * dt;
            cut_in_traj(2,i,1) = cut_in_traj(2,i-1,1) + sin(cut_in_traj(4,i-1,1)) * cut_in_traj(3,i-1,1) * dt;
            cut_in_traj(1,i,2) = cut_in_traj(1,i-1,2) + cos(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
            cut_in_traj(2,i,2) = cut_in_traj(2,i-1,2) + sin(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
        end
    elseif strcmp(Case_Code,'A4')
        % obstacle1
        o1_init = [10;3;12;0];
        o1      = obstacle(o1_init(1:2), [], o1_init(3), o1_init(4), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [30;-3;12;0];
        o2      = obstacle(o2_init(1:2), [], o2_init(3), o2_init(4), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        obs = [o1, o2];
        cut_in_traj = [];
    elseif strcmp(Case_Code,'A5')
        % obstacle1
        o1_init = [10;3;10;0];
        o1      = obstacle(o1_init(1:2), [], o1_init(3), o1_init(4), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [30;-3;12;0];
        o2      = obstacle(o2_init(1:2), [], o2_init(3), o2_init(4), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        obs = [o1, o2];
        cut_in_traj = [];
       
    elseif strcmp(Case_Code,'A6')
        % obstacle1
        o1_init = [10;3;15;0];
        o1      = obstacle(o1_init(1:2), [], o1_init(3), o1_init(4), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        % obstacle3 
        o3_init = [-5;3;15;0];
        o3      = obstacle(o3_init(1:2), [], o3_init(3), o3_init(4), NUM_CTRL, dt, t_switch);
        o3      = o3.update_traj(false);
        % obstacle2 
        o2_init = [30;-3;10;0];
        o2      = obstacle(o2_init(1:2), [], o2_init(3), o2_init(4), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        obs = [o1, o2, o3];
        cut_in_traj = [];
    elseif strcmp(Case_Code, 'B1')
        % obstacle1
        o1_init = [7;3;15;0];
        o1      = obstacle(o1_init(1:2), [], o1_init(3), o1_init(4), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        obs     = [o1];
        % start the random forward roll out for trajectory generation
        U       = zeros(2, (T+T_horizon)/dt);
        a_range = [-4, -3];
        delta_range = [-0.1, 0.1];... maximum and minimum steering change within 1 sec

        % build full target trajectory for obstacle trajectory update
        % build a full target trajectory
        cut_in_traj = o1_init * ones(1, (T+T_horizon)/dt);
        for i = 2:size(cut_in_traj,2)
            x0      = cut_in_traj(1, i-1);
            y0      = cut_in_traj(2, i-1);
            v0      = cut_in_traj(3, i-1);
            theta   = cut_in_traj(4, i-1);
            acc     = U(1,i-1);
            delta   = U(2,i-1);
            % kinematics update
            curvature = tan(delta)/o1.L;
            l = v0 * dt + 0.5 * acc * dt^2;         ... distance traveled
            cut_in_traj(3, i) = max(v0 + acc * dt, 0);  ... velocity update
            cut_in_traj(4, i) = theta + curvature * l;  ... heading update

            if (curvature == 0)
                cut_in_traj(1, i) = x0 + l * cos(theta);
                cut_in_traj(2, i) = y0 + l * sin(theta); 
            else
                cut_in_traj(1, i) = x0 + (sin(theta + curvature * l) - ...
                    sin(theta))/curvature;
                cut_in_traj(2, i) = y0 + (cos(theta) - cos(theta + ...
                    curvature * l))/curvature;
            end
            % update U
            if i > 20
                U(1, i) = (a_range(2) - a_range(1)) * rand() + a_range(1);
            else
                U(1, i)= 0;
            end
            U(2, i) = ((delta_range(2) - delta_range(1)) * rand() + delta_range(1))*dt;
%             U(2, i) = U(2, i-1) + ((delta_range(2) - delta_range(1)) * rand() + delta_range(1)) *dt;
%             U(2, i) = min(max(U(2,i),-pi/6), pi/6);
        end
    elseif strcmp(Case_Code,'B2')
        obs_init = [20;-3;8;0];
        obs1 = obstacle(obs_init(1:2), [], obs_init(3), obs_init(4), NUM_CTRL, dt, t_switch);
        obs1 = obs1.update_traj(false);
        % concatenate two obstacles 
        obs = [obs1];
        % build a full target trajectory
        cut_in_traj = obs_init * ones(1, (T+T_horizon)/dt);
        a_max = 6;
        for i = 2:size(cut_in_traj,2)
            % accelerate between 120m and 130m (i = 120:140)
            if ( i < 115 && i >= 105)... >= 105
               cut_in_traj(3,i) = cut_in_traj(3, i-1) + a_max*dt;
%             elseif (i >= 115)
%                cut_in_traj(3,i) = 12; 
            else
                cut_in_traj(3,i) = cut_in_traj(3, i-1);
            end
            cut_in_traj(1,i) = cut_in_traj(1,i-1) + cos(cut_in_traj(4,i-1)) * cut_in_traj(3,i-1) * dt;
            cut_in_traj(2,i) = cut_in_traj(2,i-1) + sin(cut_in_traj(4,i-1)) * cut_in_traj(3,i-1) * dt;
        end
    elseif strcmp(Case_Code,'Od1')
        % obstacle1
        o1_init = [40; 1.5; 0; 8; 0];
        o1_vert = inflate(o1_init);
        o1      = obstacle(o1_init(1:2), o1_vert,o1_init(3), o1_init(4), o1_init(5), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [100; -1.5; 0; 8; 0];
        o2_vert = inflate(o2_init);
        o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), o2_init(5), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        % obstacle3 
        
        
        o3_init = [40;1.5; 0; 8;0];
        o3_vert = inflate(o3_init);
        o3      = obstacle(o3_init(1:2), o3_vert, o3_init(3), o3_init(4), o3_init(5), NUM_CTRL, dt, t_switch);
        o3      = o3.update_traj(false);
        
        % obstacle4 
        o4_init = [10;-1.5;  0; 8; 0];
        o4_vert = inflate(o4_init);
        o4      = obstacle(o4_init(1:2), o4_vert, o4_init(3), o4_init(4), o4_init(5), NUM_CTRL, dt, t_switch);
        o4      = o4.update_traj(false);
        
        % obstacle5 
        o5_init = [10; 1.5;  0; 7; 0];
        o5_vert = inflate(o5_init);
        o5      = obstacle(o5_init(1:2), o5_vert, o5_init(3), o5_init(4), o5_init(5), NUM_CTRL, dt, t_switch);
        o5      = o5.update_traj(false);
        
        % obstacle6 
        o6_init = [30; 4.5;  0; 7; 0];
        o6_vert = inflate(o6_init);
        o6      = obstacle(o6_init(1:2), o6_vert, o6_init(3), o6_init(4), o6_init(5), NUM_CTRL, dt, t_switch);
        o6      = o6.update_traj(false);
        
        obs = [o1];
        
        % build a full target trajectory
%         for mm=1:size(obs)
        cut_in_traj(:,:,1) = o1_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,2) = o2_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,3) = o3_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,4) = o4_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,5) = o5_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,6) = o6_init * ones(1, (T+T_horizon)/dt);
        
%         end
       for nn=1:length(obs)
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i,nn) = cut_in_traj(1,i-1,nn) + cos(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
            cut_in_traj(2,i,nn) = cut_in_traj(2,i-1,nn) + sin(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
%             cut_in_traj(1,i,2) = cut_in_traj(1,i-1,2) + cos(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
%             cut_in_traj(2,i,2) = cut_in_traj(2,i-1,2) + sin(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
        end
       end
    elseif strcmp(Case_Code,'Od2')%we are in the last lane and we like to do a takeover
        % obstacle1
        o1_init = [20;-Lane_size/2.; 0; 8; 0];
        o1_vert = inflate(o1_init);
        o1      = obstacle(o1_init(1:2), o1_vert,o1_init(3), o1_init(4), o1_init(5), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [40; Lane_size/2.; 0; 8; 0];
        o2_vert = inflate(o2_init);
        o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), o2_init(5), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        % obstacle3 
        
        
        o3_init = [20;3*Lane_size/2.; 0; 8;0];
        o3_vert = inflate(o3_init);
        o3      = obstacle(o3_init(1:2), o3_vert, o3_init(3), o3_init(4), o3_init(5), NUM_CTRL, dt, t_switch);
        o3      = o3.update_traj(false);
        
        % obstacle4 
        o4_init = [40;-3*Lane_size/2.;  0; 8; 0];
        o4_vert = inflate(o4_init);
        o4      = obstacle(o4_init(1:2), o4_vert, o4_init(3), o4_init(4), o4_init(5), NUM_CTRL, dt, t_switch);
        o4      = o4.update_traj(false);
        
        % obstacle5 
        o5_init = [80; -Lane_size/2.;  0; 7; 0];
        o5_vert = inflate(o5_init);
        o5      = obstacle(o5_init(1:2), o5_vert, o5_init(3), o5_init(4), o5_init(5), NUM_CTRL, dt, t_switch);
        o5      = o5.update_traj(false);
        
        % obstacle6 
        o6_init = [60; Lane_size/2.;  0; 7; 0];
        o6_vert = inflate(o6_init);
        o6      = obstacle(o6_init(1:2), o6_vert, o6_init(3), o6_init(4), o6_init(5), NUM_CTRL, dt, t_switch);
        o6      = o6.update_traj(false);
        
        obs = [o1,o2,o3];
        
        % build a full target trajectory
%         for mm=1:size(obs)
        cut_in_traj(:,:,1) = o1_init * ones(1, (T+T_horizon)/dt);
        cut_in_traj(:,:,2) = o2_init * ones(1, (T+T_horizon)/dt);
         cut_in_traj(:,:,3) = o3_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,4) = o4_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,5) = o5_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,6) = o6_init * ones(1, (T+T_horizon)/dt);
        
%         end
       for nn=1:length(obs)
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i,nn) = cut_in_traj(1,i-1,nn) + cos(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
            cut_in_traj(2,i,nn) = cut_in_traj(2,i-1,nn) + sin(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
%             cut_in_traj(1,i,2) = cut_in_traj(1,i-1,2) + cos(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
%             cut_in_traj(2,i,2) = cut_in_traj(2,i-1,2) + sin(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
        end
       end
       
    elseif strcmp(Case_Code,'Od3')%we are in the last lane and we like to do a takeover
        % obstacle1
        o1_init = [80;Lane_size/2.; 0; 8; 0];
        o1_vert = inflate(o1_init);
        o1      = obstacle(o1_init(1:2), o1_vert,o1_init(3), o1_init(4), o1_init(5), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [100; Lane_size/2.; 0; 8; 0];
        o2_vert = inflate(o2_init);
        o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), o2_init(5), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        % obstacle3 
        
        
        o3_init = [20;3*Lane_size/2.; 0; 8;0];
        o3_vert = inflate(o3_init);
        o3      = obstacle(o3_init(1:2), o3_vert, o3_init(3), o3_init(4), o3_init(5), NUM_CTRL, dt, t_switch);
        o3      = o3.update_traj(false);
        
        % obstacle4 
        o4_init = [-60;Lane_size/2.;  0; 13; 0];
        o4_vert = inflate(o4_init);
        o4      = obstacle(o4_init(1:2), o4_vert, o4_init(3), o4_init(4), o4_init(5), NUM_CTRL, dt, t_switch);
        o4      = o4.update_traj(false);
        
        % obstacle5 
        o5_init = [80; -Lane_size/2.;  0; 7; 0];
        o5_vert = inflate(o5_init);
        o5      = obstacle(o5_init(1:2), o5_vert, o5_init(3), o5_init(4), o5_init(5), NUM_CTRL, dt, t_switch);
        o5      = o5.update_traj(false);
        
        % obstacle6 
        o6_init = [60; Lane_size/2.;  0; 7; 0];
        o6_vert = inflate(o6_init);
        o6      = obstacle(o6_init(1:2), o6_vert, o6_init(3), o6_init(4), o6_init(5), NUM_CTRL, dt, t_switch);
        o6      = o6.update_traj(false);
        
        obs = [o1,o2,o3,o4];
        
        % build a full target trajectory
%         for mm=1:size(obs)
        cut_in_traj(:,:,1) = o1_init * ones(1, (T+T_horizon)/dt);
        cut_in_traj(:,:,2) = o2_init * ones(1, (T+T_horizon)/dt);
         cut_in_traj(:,:,3) = o3_init * ones(1, (T+T_horizon)/dt);
           cut_in_traj(:,:,4) = o4_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,5) = o5_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,6) = o6_init * ones(1, (T+T_horizon)/dt);
        
%         end
       for nn=1:length(obs)
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i,nn) = cut_in_traj(1,i-1,nn) + cos(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
            cut_in_traj(2,i,nn) = cut_in_traj(2,i-1,nn) + sin(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
%             cut_in_traj(1,i,2) = cut_in_traj(1,i-1,2) + cos(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
%             cut_in_traj(2,i,2) = cut_in_traj(2,i-1,2) + sin(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
        end
       end
       
       elseif strcmp(Case_Code,'Od4')%a fast car Vtgt>Vroad s chasing the ego car and will not stop
        % obstacle1
        o1_init = [-60;3*Lane_size/2.; 0; 19; 0];
        o1_vert = inflate(o1_init);
        o1      = obstacle(o1_init(1:2), o1_vert,o1_init(3), o1_init(4), o1_init(5), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [100; Lane_size/2.; 0; 8; 0];
        o2_vert = inflate(o2_init);
        o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), o2_init(5), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        % obstacle3 
        
        
        o3_init = [20;3*Lane_size/2.; 0; 8;0];
        o3_vert = inflate(o3_init);
        o3      = obstacle(o3_init(1:2), o3_vert, o3_init(3), o3_init(4), o3_init(5), NUM_CTRL, dt, t_switch);
        o3      = o3.update_traj(false);
        
        % obstacle4 
        o4_init = [-60;Lane_size/2.;  0; 13; 0];
        o4_vert = inflate(o4_init);
        o4      = obstacle(o4_init(1:2), o4_vert, o4_init(3), o4_init(4), o4_init(5), NUM_CTRL, dt, t_switch);
        o4      = o4.update_traj(false);
        
        % obstacle5 
        o5_init = [80; -Lane_size/2.;  0; 7; 0];
        o5_vert = inflate(o5_init);
        o5      = obstacle(o5_init(1:2), o5_vert, o5_init(3), o5_init(4), o5_init(5), NUM_CTRL, dt, t_switch);
        o5      = o5.update_traj(false);
        
        % obstacle6 
        o6_init = [60; Lane_size/2.;  0; 7; 0];
        o6_vert = inflate(o6_init);
        o6      = obstacle(o6_init(1:2), o6_vert, o6_init(3), o6_init(4), o6_init(5), NUM_CTRL, dt, t_switch);
        o6      = o6.update_traj(false);
        
        obs = [];
        
        % build a full target trajectory
%         for mm=1:size(obs)
        cut_in_traj(:,:,1) = o1_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,2) = o2_init * ones(1, (T+T_horizon)/dt);
%          cut_in_traj(:,:,3) = o3_init * ones(1, (T+T_horizon)/dt);
%           cut_in_traj(:,:,4) = o4_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,5) = o5_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,6) = o6_init * ones(1, (T+T_horizon)/dt);
        
%         end
       for nn=1:length(obs)
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i,nn) = cut_in_traj(1,i-1,nn) + cos(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
            cut_in_traj(2,i,nn) = cut_in_traj(2,i-1,nn) + sin(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
%             cut_in_traj(1,i,2) = cut_in_traj(1,i-1,2) + cos(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
%             cut_in_traj(2,i,2) = cut_in_traj(2,i-1,2) + sin(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
        end
       end
       elseif strcmp(Case_Code,'Od5')%for two lane change tests
        % obstacle1
        o1_init = [-60;3*Lane_size/2.; 0; 19; 0];
        o1_vert = inflate(o1_init);
        o1      = obstacle(o1_init(1:2), o1_vert,o1_init(3), o1_init(4), o1_init(5), NUM_CTRL, dt, t_switch);
        o1      = o1.update_traj(false);
        
        % obstacle2 
        o2_init = [100; Lane_size/2.; 0; 8; 0];
        o2_vert = inflate(o2_init);
        o2      = obstacle(o2_init(1:2), o2_vert, o2_init(3), o2_init(4), o2_init(5), NUM_CTRL, dt, t_switch);
        o2      = o2.update_traj(false);
        
        % obstacle3 
        
        
        o3_init = [20;3*Lane_size/2.; 0; 8;0];
        o3_vert = inflate(o3_init);
        o3      = obstacle(o3_init(1:2), o3_vert, o3_init(3), o3_init(4), o3_init(5), NUM_CTRL, dt, t_switch);
        o3      = o3.update_traj(false);
        
        % obstacle4 
        o4_init = [-60;Lane_size/2.;  0; 13; 0];
        o4_vert = inflate(o4_init);
        o4      = obstacle(o4_init(1:2), o4_vert, o4_init(3), o4_init(4), o4_init(5), NUM_CTRL, dt, t_switch);
        o4      = o4.update_traj(false);
        
        % obstacle5 
        o5_init = [80; -Lane_size/2.;  0; 7; 0];
        o5_vert = inflate(o5_init);
        o5      = obstacle(o5_init(1:2), o5_vert, o5_init(3), o5_init(4), o5_init(5), NUM_CTRL, dt, t_switch);
        o5      = o5.update_traj(false);
        
        % obstacle6 
        o6_init = [60; Lane_size/2.;  0; 7; 0];
        o6_vert = inflate(o6_init);
        o6      = obstacle(o6_init(1:2), o6_vert, o6_init(3), o6_init(4), o6_init(5), NUM_CTRL, dt, t_switch);
        o6      = o6.update_traj(false);
        
        obs = [];
        
        % build a full target trajectory
%         for mm=1:size(obs)
        cut_in_traj(:,:,1) = o1_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,2) = o2_init * ones(1, (T+T_horizon)/dt);
%          cut_in_traj(:,:,3) = o3_init * ones(1, (T+T_horizon)/dt);
%           cut_in_traj(:,:,4) = o4_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,5) = o5_init * ones(1, (T+T_horizon)/dt);
%         cut_in_traj(:,:,6) = o6_init * ones(1, (T+T_horizon)/dt);
        
%         end
       for nn=1:length(obs)
        for i = 2:size(cut_in_traj,2)
            cut_in_traj(1,i,nn) = cut_in_traj(1,i-1,nn) + cos(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
            cut_in_traj(2,i,nn) = cut_in_traj(2,i-1,nn) + sin(cut_in_traj(5,i-1,nn)) * cut_in_traj(4,i-1,nn) * dt;
%             cut_in_traj(1,i,2) = cut_in_traj(1,i-1,2) + cos(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
%             cut_in_traj(2,i,2) = cut_in_traj(2,i-1,2) + sin(cut_in_traj(4,i-1,2)) * cut_in_traj(3,i-1,2) * dt;
        end
       end
    elseif strcmp(Case_Code,'S1')
        obs = [];
        cut_in_traj = [];
    end
end