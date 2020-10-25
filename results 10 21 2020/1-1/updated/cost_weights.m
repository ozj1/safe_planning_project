global acc_mat
acc_mat  = [1 0;0 0];

global yawr_mat
yawr_mat = [0 0;0 1];

% weights for state cost function 
global w_jerk w_acc w_del w_ref w_vel w_end_ref w_end_acc w_end_vel

% weights for obstacle avoidance
global q1_back q2_back
global q1_front q2_front
global q1_road q2_road
global q1_CenterLane q2_CenterLane %added by Omid
global v_min q1_min_vel q2_min_vel%added by Omid

global q1_jerk q2_jerk q1_acc q2_acc q1_del q2_del
global NUM_CTRL;                    ... number of controls



%% D1 yields and ACC
% w_jerk   =10;
% w_acc       = 10;
% w_del       = 30;
% % w_ref       = 10;... 3
% % w_vel       = 0.1;
% w_ref = 20.* ones(NUM_CTRL,1);
% w_vel = 0.1* ones(NUM_CTRL,1);
% 
% w_end_ref   = 25;
% w_end_acc   = 10;
% w_end_vel   = 10;
% 
% q1_back     = 30;
% q2_back     = 30;
% 
% q1_front    = 30;
% q2_front    = 30;
% 
% q1_road = 10;
% q2_road = 10;
% 
% q1_min_vel =5;
% q2_min_vel=5;
% 
% q1_CenterLane=0;
% q2_CenterLane=0;


%% A1 overtakes 
% w_acc       = 2;
% w_del       = 5;
% w_ref       = 15;
% w_vel       = 0.1;
% 
% w_end_ref   = 30;
% w_end_vel   = 3;
% 
% q1_back     = 10;
% q2_back     = 10;
% 
% q1_front    = 10;
% q2_front    = 10;
% 
% q1_road = 5;
% q2_road = 5;

%% A2 ACC
% w_acc       = 10;... 2
% w_del       = 50;... 5
% w_ref       = 30;... 30
% w_vel       = 0.1;... 0.1
% 
% w_end_ref   = 30;
% w_end_vel   = 3;
% 
% q1_back     = 20;
% q2_back     = 10;
% 
% q1_front    = 20;
% q2_front    = 10;
% 
% q1_road = 0;
% q2_road = 5;

% reserved for stop sign case
global q1_stop q2_stop
q1_stop = 20;
q2_stop = 1;

% %% A3 overtakes and avoid 
% w_acc       = 10;
% w_del       = 30;
% w_ref       = 0.1;
% w_vel       = 10;
% 
% w_end_ref   = 10;
% w_end_vel   = 10;
% 
% q1_back     = 20;
% q2_back     = 10;
% 
% q1_front    = 20;
% q2_front    = 10;
% 
% q1_road = 5;
% q2_road =5;

%%%%%%%%%%%%%%

% % A2 ACC, B2 overtake then yield back
% w_acc       = 10;
% w_del       = 30;
% w_ref       = 20;... 3
% w_vel       = 0.1;

% w_end_ref   = 20;
% w_end_ref   = 20;
% w_end_vel   = 20;

% w_end_ref = w_ref;
% w_end_vel = w_vel;

% % barrier function terms
% global t_bar w_bar
% t_bar       = 5;
% w_bar       = 0;... 10

% global q1_back q2_back
% q1_back     = 30;
% q2_back     = 10;

% q1_back     = 50;
% q2_back     = 10;

% global q1_front q2_front
% q1_front    = 30;
% q2_front    = 10;

% q1_front     = 50;
% q2_front     = 10;

%% working weight with d_safe of 15m 12m/s initial 
% global w_acc w_del w_ref w_vel
% w_acc       = 2;
% w_del       = 5;
% w_ref       = 5;
% w_vel       = 1;
% 
% % barrier function terms
% global t_bar w_bar
% t_bar       = 5;
% w_bar       = 10;


%% Od1 overtakes and avoid 
% w_jerk   =10;
% w_del       = 10;
% % w_ref       = 10;
% % w_vel       = 0.1;
% w_acc       = 10;
% 
% w_end_ref   = 50;
% w_end_vel   = 10;
% 
% q1_back     = 10;
% q2_back     =10;
% 
% q1_front    =10;
% q2_front    = 10;
% 
% q1_road = 50;
% q2_road =50;
% 
% q1_CenterLane=10;
% q2_CenterLane=10;

%% Od2 overtakes and avoid 
% w_jerk   =0.05* ones(NUM_CTRL,1);
% w_acc       = 10* ones(NUM_CTRL,1);
% w_del       = 30* ones(NUM_CTRL,1);
% % w_ref       = 10;... 3
% % w_vel       = 0.1;
% w_ref = 20.* ones(NUM_CTRL,1);
% w_vel = 0.1* ones(NUM_CTRL,1);
q1_jerk=0.05;%10
q2_jerk=0.05;%10
q1_acc = 3;%3
q2_acc = 3;%3
q1_del = 50;%50
q2_del = 50;%50

w_end_ref   = 50;%50
w_end_acc   = 10;%110
w_end_vel   = 10;%10

q1_back     = 50;
q2_back     = 50;

q1_front    = 50;
q2_front    = 50;

q1_road = 50;
q2_road = 50;

q1_min_vel =0;
q2_min_vel=0;

q1_CenterLane=0;
q2_CenterLane=0;

%% adaptive weight calculation
global a_vel b_vel a_ref b_ref
a_vel =  1.4167;
b_vel = -0.0916;

a_ref =  0.2024;
b_ref = -0.0916;