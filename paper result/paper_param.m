%% D1 final parameters
% initialization
X_start(3)  = 12;   ... initial velocity of 14.0m/sec
    
% weights
w_acc       = 20;
w_del       = 30;
w_ref       = 10;... 3
w_vel       = 0.1;

w_end_ref   = 10;
w_end_vel   = 5;

q1_back     = 30;
q2_back     = 10;

q1_front    = 30;
q2_front    = 10;

% scenario
obs1 = obstacle([5;-3], [], 12, 0, NUM_CTRL, dt, t_switch);
obs1 = obs1.update_ctrl(zeros(U_DIM, NUM_CTRL));
% target performing lane change
% last four inputs: (type, velocity, tf, x0)
cut_in_traj = (reftraj_gen(T+T_horizon, dt, -3, 'LaneChange', 12, 3, 5))'; 
obs1 = obs1.load_traj(cut_in_traj(:,1:1+NUM_CTRL), [], false);
% compile into an obstacle 
obs = [obs1];

% plotting 
subplot(2, 1, 1);
interval = 10;
i0 = 1;
it = 80;
% environment visualizations
plot(ref_traj(:,1),ref_traj(:,2),'-.k');  
hold on
plot(side_lane(:,1),side_lane(:,2),'-.k');  
hold on
plot(left_road(:,1),left_road(:,2),'k');  
hold on
plot(right_road(:,1),right_road(:,2),'k');  
hold on
plot(mid_road(:,1),mid_road(:,2),'k');  
hold on
% draw past trajectory
plot(X_planned(1,1:i0), X_planned(2,1:i0), 'b');
hold on
% draw target trajectory
plot(obs_traj(1,1:i0), obs_traj(2,1:i0), 'r');
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
end

% draw past trajectory
plot(X_planned(1,:), X_planned(2,:), 'b');
hold on
% draw target trajectory
plot(obs_traj(1,:), obs_traj(2,:), 'r');
hold off

axis equal
xlim([0 80]); % B2
ylim([-8 8]);

% velocity profile
subplot(2, 1, 2);
yyaxis left
plot(X_planned(3, i0:it));
ylabel('Velocity (m/s)');
xlabel('longitudinal displacement (m)');

yyaxis right
plot(U_planned(1,:));
ylabel('Acceleration (m/s^2)');
xlim([0 80]);
grid on

%% A1 final parameters
w_acc       = 10;
w_del       = 30;
w_ref       = 20;... 3
w_vel       = 0.1;

w_end_ref   = 10;
w_end_vel   = 5;

q1_back     = 30;
q2_back     = 10;

q1_front    = 30;
q2_front    = 10;

X_start(3)  = 10;   ... initial velocity of 14.0m/sec

% plot
figure()

subplot(2, 1, 1);
interval = 5;
i0 = 41;
it = 140;
% environment visualizations
plot(ref_traj(:,1),ref_traj(:,2),'-.k');  
hold on
plot(side_lane(:,1),side_lane(:,2),'-.k');  
hold on
plot(left_road(:,1),left_road(:,2),'k');  
hold on
plot(right_road(:,1),right_road(:,2),'k');  
hold on
plot(mid_road(:,1),mid_road(:,2),'k');  
hold on
% draw past trajectory
plot(X_planned(1,1:i0), X_planned(2,1:i0), 'b');
hold on
% draw target trajectory
plot(obs_traj(1,1:i0), obs_traj(2,1:i0), 'r');
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
end

% draw past trajectory
plot(X_planned(1,it-1:end), X_planned(2,it-1:end), 'b');
hold on
% draw target trajectory
plot(obs_traj(1,it-1:end), obs_traj(2,it-1:end), 'r');
hold off

axis equal
xlim([0 80]); % B2
ylim([-8 8]);

% velocity profile
subplot(2, 1, 2);
yyaxis left
plot(X_planned(3, i0:it));
ylabel('Velocity (m/s)');
xlabel('longitudinal displacement (m)');

yyaxis right
plot(U_planned(1,:));
ylabel('Acceleration (m/s^2)');
xlim([0 80]);
grid on

% set plot position
x0 = 0;
y0 = 0;
width = 1920;
height = 1080;
set(gcf,'position',[x0,y0,width,height]);

%% B2 weights
w_acc       = 20;
w_del       = 30;
w_ref       = 10;... 3
w_vel       = 0.1;

w_end_ref   = 10;
w_end_vel   = 5;

q1_back     = 30;
q2_back     = 10;

q1_front     = 30;
q2_front     = 10;

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

% no reach plotting
figure()

subplot(2, 1, 1);
interval = 5;
i0 = 50;
it = 160;
% environment visualizations
plot(side_lane(:,1),side_lane(:,2)+6,'--r');  
hold on
plot(ref_traj(:,1),ref_traj(:,2),'-.k');  
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
    if i >= 110 && i < 125
        % draw prediction
        plot(X_local(1,1:5,i), X_local(2,1:5,i), 'b--*', 'LineWidth', 0.1);
        hold on

        % draw reachability
        for j = 1:5
            plot([obs_short(1,:,j,i) obs_short(1,1,j,i)], [obs_short(2,:,j,i),obs_short(2,1,j,i)], 'g','LineWidth', 2);
        end
    end
end

% draw past trajectory
plot(X_planned(1,:), X_planned(2,:), 'b');
hold on
% draw target trajectory
plot(obs_traj(1,:), obs_traj(2,:), 'r');
hold off

axis equal
xlim([i0 it]); % B2
ylim([-8 8]);

% velocity profile
subplot(2, 1, 2);
yyaxis left
plot(obs_traj(3,:), 'm', 'LineWidth', 2);
hold on
plot(X_planned(3, :), '-b', 'LineWidth', 2);
ylabel('Velocity (m/s)');
xlabel('longitudinal displacement (m)');
ylim([7 18])

yyaxis right
plot(U_planned(1,:), 'LineWidth', 2);
ylabel('Acceleration (m/s^2)');
xlim([i0 it]);
grid on
legend('target vehicle', 'ego vehicle', 'acceleration');

% set plot position
x0 = 0;
y0 = 0;
width = 1920;
height = 1080;
set(gcf,'position',[x0,y0,width,height]);