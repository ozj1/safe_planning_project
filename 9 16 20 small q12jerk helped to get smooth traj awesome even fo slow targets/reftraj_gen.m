function ref = reftraj_gen(T_tot, dt, y_init,x_init, varargin)
% ref_traj    = reftraj_gen(2*T, dt, -1.5, 'Linear', vref);%it must be 3 for D1 and -3 for others 

% This function is for reference trajectory generation
% Inputs
% ======
% T     : total time horizon
% dt    : time interval
% opt   : reference trajectory type (default as linear)
%   accepted input types:
%       1) 'Linear'         : linear reference trajectory along x-axis 
%       2) 'LaneChange'     : a lane change like trajectory modeled by a 
%       high order polynomial
%       3) 'SelfDefined', fn: self defined trajectory with specified data
%       file. The first column is x-coord, second column being y-coord and
%       the last being reference velocity at each point

%     vehicle_parameter;
%     L = param.lf + param.lr;

    % Check number of inputs.
    numvarargs = length(varargin);
    if numvarargs > 5% changed by Omid it was 4 previously 
        error('myfuns:reftraj_Gen:TooManyInputs', ...
            'requires at most 1 optional input');
    end

    global Lane_size
    global NUM_TOTAL
%     global NUM_CTRL
    % required for weight calculation in DYNCST 
    global y_final y_temp_final y_temp_init traj_count
    % setup default fields
    optargs = {'Linear' '' '' '3' '20'};%Omid: I changed {'Linear' '' '3' '20'} to {'Linear' '' '3' '0'}

    % fill in the cell with incoming data
    optargs(1:numvarargs) = varargin;
    
    type    = cell2mat(optargs(1));           ... reference trajectory type
    tgt_acc = cell2mat(optargs(2)); ... target reference velocity 
    tgt_vel = cell2mat(optargs(3)); ... target reference velocity 
    tf = cell2mat(optargs(4));   %for D1   ... lane changing finish time %Omid: I added str2num to change the type of character to number
%     tf = str2num(cell2mat(optargs(4)));      ... lane changing finish time %Omid: I added str2num to change the type of character to number
%     x0 = str2num(cell2mat(optargs(5)));      ... initial x position
    x0 = cell2mat(optargs(5));  %for D1    ... initial x position

    % linear trajectory along x-axis
    if strcmp(type,'Linear')
        v_des     = tgt_vel;
        a_des=tgt_acc;
        interval  = v_des * dt;
        x_ref     = 0:interval:interval*(NUM_TOTAL);
        y_ref     = y_init*ones(size(x_ref));
        a_ref     = a_des * ones(size(x_ref));
        v_ref     = v_des * ones(size(x_ref));
        theta_ref = zeros(size(x_ref));%Omid all tetha qual to zero as it is linear
        ref       = [x_ref' y_ref' a_ref' v_ref' theta_ref'];
    
    % Lane Changing trajectory
    elseif strcmp(type,'LaneChange')
        t0 = 0;
        
        v_des   = tgt_vel;
        a_des=tgt_acc;
        % corresponds to x0 dx0 ddx0 xf dxf ddxf
        qx      = [x0, v_des, 0, x0+v_des*tf, v_des, 0];
        % corresponds to y0 dy0 ddy0 yf dyf ddyf
        if (y_init == -Lane_size)
            y_final = Lane_size;
        elseif (y_init == Lane_size)
            y_final = -Lane_size;
        elseif (y_init == Lane_size/2.)%below is added by Omid
            y_final = -Lane_size/2.;
        elseif (y_init == -Lane_size/2.)
            y_final = Lane_size/2.;
        elseif (y_init == -3*Lane_size/2.)
            y_final = 3*Lane_size/2.;%Omid: it should be -Lane_size/2. for one lane changing
        elseif (y_init == 3*Lane_size/2.)
            y_final = 1*Lane_size/2.;%Omid: it should be -Lane_size/2. for one lane changing
        end
        steps=(y_final-y_init)/Lane_size;

        if (traj_count<=abs(steps))
            sign=(steps/abs(steps));
            y_temp_init=y_init+(traj_count-1)*sign*Lane_size;
            y_temp_final=y_init+(traj_count)*sign*Lane_size;
             
        qy= [y_temp_init,0, 0, y_temp_final, 0,     0]; 

        T_mat = [1, t0, t0^2, t0^3,   t0^4,    t0^5;
                 0, 1,  2*t0, 3*t0^2, 4*t0^3,  5*t0^4;
                 0, 0,  2,    6*t0,   12*t0^2, 20*t0^3;
                 1, tf, tf^2, tf^3,   tf^4,    tf^5;
                 0, 1,  2*tf, 3*tf^2, 4*tf^3,  5*tf^4;
                 0, 0,  2,    6*tf,   12*tf^2, 20*tf^3];
             
        % 6x1 column vec
        a     = inv(T_mat)*qx';
        b     = inv(T_mat)*qy';
        t_now = @(t) [1, t, t^2, t^3, t^4, t^5]';
        x_now = @(t) a'*t_now(t);
        y_now = @(t) b'*t_now(t);
        
        n         = T_tot/dt;
        x_ref     = zeros(n+1,1);
        y_ref     = zeros(n+1,1);
        a_ref     = a_des * ones(n+1,1);
        v_ref     = v_des * ones(n+1,1);
        theta_ref = zeros(n+1,1);
        
        tspan = 0:dt:tf;
        for i = 1:length(tspan)  
            x_ref(i)     = x_now(tspan(i));
            y_ref(i)     = y_now(tspan(i));
        end
        
        yf = y_ref(i);
        
        for i = i+1 : n+1
            x_ref(i)    = x_ref(i-1) + dt * v_des;
            y_ref(i)    = yf;
        end
        
        for i = 1:n  
            theta_ref(i) = atan2(y_ref(i+1)-y_ref(i), x_ref(i+1)-x_ref(i));
        end
        theta_ref(end) = 0;
        
        ref   = [x_ref y_ref a_ref v_ref theta_ref];
    end
    % self defined data loading
    elseif strcmp(type,'SelfDefined')
        data      = load(filename);
        x_ref     = data(:,1);
        y_ref     = data(:,2);
        a_ref     = data(:,3);
        v_ref     = data(:,4);
        theta_ref = data(:,5);
        ref       = [x_ref' y_ref' a_ref' v_ref' theta_ref'];
    else
        error('myfuns:reftraj_Gen:InvalidTypeInputs', ...
            'see help for allowed inputs');
    end

end