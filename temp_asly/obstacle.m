classdef obstacle
% This is the obstacle class. It has the following member functions:
%
% constructor
% ===========
% input: 
%   center0     -- initial starting center of obstacle
%   vertices    -- the object vertices at the initial state
%   v0          -- the initial velocity of the object
%   yaw0        -- the initial heading angle of the object
%   num_ctrl    -- number of control in the planning horizon
%   dt          -- discretization time step
%   w_c         -- the cost function weight for the closest distance term
%   ds          -- safety distance for the object
%   t           -- log barrier function weight
% 
% In this function, an empty trajectory, an empty vertices matrix and zero 
% cost function derivatives are initialized along with physical parameters.
% Most importantly, the set of vertices that is centered at the origin with
% zero heading angle is recovered
%
% update_ctrl
% ===========
% input:
%   ctrl        -- control sequence of the obstacle in the next horizon
%
% In this function, only control sequence is updated
%
% update_traj
% ===========
% In this function, all states and vertices are updated based on the
% control sequence set previously
%
% next_window
% ===========
% input:
%   ctrl        -- control sequence of the obstacle in the next horizon
%
% In this function, the initial state and vertice set are first initialized
% as the second element in the previous iteration. Then, update_traj is
% called.
%
% load_traj
% =========
% In this function, a trajectory is loaded and corresponding vertices are
% calculated and updated
%
% calc_cost (WIP)
% =========
% In this function, all cost function derivatives are calculated and
% updated. 
% I might use finite difference b/c the analytical form of might be hard to
% solve


    %% class member variables
    properties
        vertices0   % polygon vertices when centering on the origin 
                    % (repeating the first vertex)
        vertices    % veritces at each step
        
        center0     % initial center position, two element array (2x1)
        a0          % initial velocity
        v0          % initial velocity
        yaw0        % initial yaw rate
        
        traj_pred   % predicted trajectory
        traj        % obstacle trajectory (x,y,vel,yaw)
        ctrl        % obstacle control sequence (U_DIM * NUM_CTRL)
        
        num_ctrl    % number of sliding window length
        dt          % discretization time step
        t_switch    % switch between short term and long term prediction
        
        L           % obstacle wheel base
        
        dist_safe   % obstacle avoidance safe distance
        
        ellipse_axis% obstacle ellipse axis
    end
    
    %% class member constants
    properties (Constant)
        
         X_DIM = 5;      % state vector dimension
         U_DIM = 2;      % input vector dimension
        len   = 5.0;    % vehicle length
        wid   = 2.0;    % vehicle width
        lf    = 1.5;    % Vehicle front wheel distance to CG
        lr    = 1.5;    % Vehicle rear wheel distance to CG
        rad   = 1.7493; % Vehicle two-circle representation circle radius
        
        t_safe      = 3.0;   % safe time, second
    end
    
    %% member functions
    methods
        %% constructor
        function obj = obstacle(center0, vertices,a0, v0, yaw0, ...
                num_ctrl, dt, t_switch)
            
            rotmat = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % geometry
            obj.center0     = center0;
            if (~isempty(vertices))
                obj.vertices0   = rotmat(-yaw0) * ...
                    (vertices - center0 * ones(1, size(vertices, 2)));... recover vertices0
                obj.vertices    = zeros([size(vertices), num_ctrl+1]);
                obj.vertices(:,:, 1) = vertices;... initialize the first set of vertices
            end
            % initialize the semi major and minor axis for the ellipse 
            obj.ellipse_axis = diag([obj.len/2*sqrt(2) + obj.len/2, 
                                    obj.wid/2*sqrt(2)+ obj.wid/2]) * ones(2, num_ctrl+1);
            % states initialization
            obj.a0          = a0;
            obj.v0          = v0;
            obj.yaw0        = yaw0;
            
            obj.num_ctrl    = num_ctrl;
            obj.dt          = dt;
            obj.t_switch    = t_switch;
%             obj.dist_safe   = ds;
            
            % trajectory initialization
            obj.traj        = zeros(obj.X_DIM, num_ctrl + 1);
            obj.traj(:,1)   = [center0(1); center0(2); a0; v0; yaw0];
            obj.ctrl        = zeros(obj.U_DIM, num_ctrl);
            
            % update predicted trajectory
            obj.traj_pred   = zeros(obj.X_DIM, num_ctrl + 1);
            
            obj.L = obj.lf + obj.lr;
        end
        
        %% update control sequence
        function obj = update_ctrl(obj, ctrl)
            obj.ctrl = ctrl;
        end
        
        %% update trajectory based on self control sequence and state
        function obj = update_traj(obj, flag)
            % flag: whether to update vertices
            
            rotmat = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            for i = 2:obj.num_ctrl + 1
                X  = obj.traj(:, i-1);
                U  = obj.ctrl(:, i-1);
                
                % extract information for convenience
                x       = X(1);
                y       = X(2);
                acc       = X(3);
                v       = X(4);
                theta   = X(5);
                jerk     = U(1);
                delta   = U(2);
                
                curvature   = tan(delta) / obj.L;
                l           = v * obj.dt + 0.5 * acc * (obj.dt)^2 +(1/6.)*jerk*(obj.dt)^3;
                
                % update vehicle state via kinematics
               
                obj.traj(4, i) = v + acc * obj.dt+0.5 * jerk * (obj.dt)^2;... velocity update
                obj.traj(3, i) = acc +jerk* obj.dt;... acc update
                obj.traj(5, i) = theta + curvature * l;%edited b Omid
                
                
                if (curvature == 0)
                    obj.traj(1, i) = x + l * cos(theta);
                    obj.traj(2, i) = y + l * sin(theta);
                else 
                    obj.traj(1, i) = x + (sin(theta + curvature * l) - ...
                        sin(theta))/curvature;
                    obj.traj(2, i) = y + (cos(theta) - cos(theta + ...
                        curvature * l))/curvature;
                end
                
                % update vertices 
                if (flag)
                    obj.vertices(:, :, i) = rotmat(theta) * obj.vertices(:, :, 1) + ...
                        ([x;y] - obj.center0)*ones(1,size(obj.vertices0, 2));
                end
            end
        end
        
        %% slide the obstacle to the next window
        function obj = next_window(obj, ctrl, flag, bounding_box)
            % flag: whether to update the obstacle vertices
            % bounding_box: 2x4xk vertices of the reachability set
            
            if (~isempty(ctrl))
                obj = obj.update_ctrl(ctrl);
            end
            
            % update the first state on the trajectory
            obj.traj(:,1) = obj.traj(:,2);
            
            if (flag)
                % update the first set of vertices
                obj.vertices(:,:,1) = obj.vertices(:,:,2);            
            end
            
            obj = obj.update_traj(flag);
            
            % update the elliptical axis length if bounding box is given
            if (~isempty(bounding_box))
                for i = 1:(obj.t_switch/obj.dt)
                    point1 = bounding_box(:,1,i);
                    point2 = bounding_box(:,2,i);
                    point3 = bounding_box(:,3,i);
                    if (point1(1) == point2(1))
                        dx = abs(point2(1) - point3(1));
                        dy = abs(point2(2) - point1(2));
                    else
                        dx = abs(point2(1) - point1(1));
                        dy = abs(point2(2) - point3(2));
                    end
                    obj.ellipse_axis(:,i) = [dx*sqrt(2)/2 + obj.len/2; 
                                             dy*sqrt(2)/2 + obj.wid/2];
                end
            end
        end
        
        %% load a trajectory as the self trajectory and update bounding box
        function obj = load_traj(obj, traj, bounding_box, flag)
            rotmat = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % trajectory is defined by 4xn matrix
            obj.traj = traj;
            for i = 1:obj.num_ctrl+1
                x       = obj.traj(1, i);
                y       = obj.traj(2, i);
                theta   = obj.traj(3, i); 
                if (flag == true)
                    % upate vertices 
                    obj.vertices(:, :, i) = rotmat(theta) * obj.vertices0 + ...
                        ([x;y] - obj.center0)*ones(1,size(obj.vertices0, 2));
                end
            end
            % update the elliptical axis length if bounding box is given
            if (~isempty(bounding_box))
                for i = 1:(obj.t_switch/obj.dt)
                    point1 = bounding_box(:,1,i);
                    point2 = bounding_box(:,2,i);
                    point3 = bounding_box(:,3,i);
                    if (point1(1) == point2(1))
                        dx = abs(point2(1) - point3(1));
                        dy = abs(point2(2) - point1(2));
                        % also update the trajectory
                        obj.traj(1,i) = (point2(1) + point3(1))/2;
                        obj.traj(2,i) = (point2(2) - point1(2))/2;
                    else
                        dx = abs(point2(1) - point1(1));
                        dy = abs(point2(2) - point3(2));
                        % also update the trajectory
                        obj.traj(1,i) = (point2(1) - point1(1))/2;
                        obj.traj(2,i) = (point2(2) - point3(2))/2;
                    end
                    obj.ellipse_axis(:,i) = [dx*sqrt(2)/2 + obj.len/2; 
                                             dy*sqrt(2)/2 + obj.wid/2];
                end
            end
        end
        
        %% load a trajectory as the self trajectory and update bounding box
        function obj = load_traj_predicted(obj, traj_pred, bounding_box, flag)
            rotmat = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];
            % trajectory is defined by 4xn matrix
            obj.traj_pred = traj_pred;
            for i = 1:obj.num_ctrl+1
                x       = obj.traj_pred(1, i);
                y       = obj.traj_pred(2, i);
                theta   = obj.traj_pred(3, i); 
                if (flag == true)
                    % upate vertices 
                    obj.vertices(:, :, i) = rotmat(theta) * obj.vertices0 + ...
                        ([x;y] - obj.center0)*ones(1,size(obj.vertices0, 2));
                end
            end
            % update the elliptical axis length if bounding box is given
            if (~isempty(bounding_box))
                for i = 1:(obj.t_switch/obj.dt)
%                 for i = 1:(obj.t_switch/(2*obj.dt))%Omid
                    point1 = bounding_box(:,1,i);
                    point2 = bounding_box(:,2,i);
                    point3 = bounding_box(:,3,i);
                    if (point1(1) == point2(1))
                        dx = abs(point2(1) - point3(1));
                        dy = abs(point2(2) - point1(2));
                        % also update the trajectory
                        obj.traj_pred(1,i) = (point2(1) + point3(1))/2;
                        obj.traj_pred(2,i) = (point2(2) + point1(2))/2;
                    else
                        dx = abs(point2(1) - point1(1));
                        dy = abs(point2(2) - point3(2));
                        % also update the trajectory
                        obj.traj_pred(1,i) = (point2(1) + point1(1))/2;
                        obj.traj_pred(2,i) = (point2(2) + point3(2))/2;
                    end
                    obj.traj_pred(4,i) = 0;
                    obj.ellipse_axis(:,i) = [dx*sqrt(2)/2 + obj.len/2; 
                                             dy*sqrt(2)/2 + obj.wid/2];
                end
            end
        end
        
    end
end