global param
param.m     = 1573.;     % Vehicle mass
param.Iz    = 2873.;     % Vehicle z-axis rotational inertia
param.lf    = 1.5;      % Vehicle front wheel distance to CG
param.lr    = 1.5;      % Vehicle rear wheel distance to CG
param.L     = param.lf + param.lr;
param.len   = 5.;        % Vehicle total length
param.wid   = 2.0;      % Vehicle totale width
param.rad   = 1.7493;   % Vehicle two-circle representation circle radius
param.C_af  = 80000.;    % Cornering stiffness of each front tire
param.C_ar  = 80000.;    % Cornering stiffness of each rear tire


global a_dot_max a_dot_min v_dot_max v_dot_min delta_max delta_min v_min

a_dot_max = 0.9;          % maximum jerk rate, m/s^3
a_dot_min = -0.9;         % maximum deceleration rate, m/s^3

v_dot_max = 6.;          % maximum acceleration rate, m/s^2
v_dot_min = -4.;         % maximum deceleration rate, m/s^2

delta_max = pi/9.;       % maximum steering angle
delta_min = -pi/9.;      % minimum steering angle

v_min=1.;

%road globad values- this part added b Omid
global road_up_lim road_low_lim Lane_size
road_up_lim =3.;
road_low_lim=-3.;
Lane_size=3.;

%for tailgate
global min_safe_dis
min_safe_dis=0.;