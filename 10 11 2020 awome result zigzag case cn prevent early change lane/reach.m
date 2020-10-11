function [x_flow, y_flow] = reach(x, model)

global v_dot_max v_dot_min 

x_3d = [x(1,:);x(2,:);x(5,:)];
u = [x(4,:);0];%why is this 0? Omid %it was u = [x(4,:);0]; changed by Omid to u = [x(4,:);x(5,:)]
x_UNCERTAINTY = 0.01;
y_UNCERTAINTY = 0.01;
v_UNCERTAINTY = 0.01;
phi_UNCERTAINTY = 0.01;
delta_UNCERTAINTY = 0.001;
v_noise_max = v_dot_max ;
v_noise_min = v_dot_min ;
delta_noise_max = 0.1;
delta_noise_min = -0.1;
PRECISION = 4;
l = 5; %length of a car
a13 = pc(-1*u(1)*sin(x_3d(3)), PRECISION);
a23 = pc(u(1)*cos(x_3d(3)), PRECISION);
b11 = pc(cos(x_3d(3)), PRECISION);
b21 = pc(sin(x_3d(3)), PRECISION);
b31 = pc(tan(u(2))/l, PRECISION);
b32 = pc(u(1)/(l*cos(u(2))^2), PRECISION);
parameter_str_linear = a13+" "+a23+" "+b11+" "+b21+" "+b31+" "+b32;
x0 = x_3d(1);
x0_interval = [pc(x_3d(1)-x_UNCERTAINTY-x0, PRECISION), pc(x_3d(1)-x_UNCERTAINTY-x0, PRECISION)];
y0 = x_3d(2);
y0_interval = [pc(x_3d(2)-y_UNCERTAINTY-y0, PRECISION), pc(x_3d(2)-y_UNCERTAINTY-y0, PRECISION)];
phi0 = x_3d(3);
phi0_interval = [pc(x_3d(3)-phi_UNCERTAINTY-phi0, PRECISION), pc(x_3d(3)-phi_UNCERTAINTY-phi0, PRECISION)];
v0_interval = [pc(u(1)-v_UNCERTAINTY, PRECISION), pc(u(1)+v_UNCERTAINTY, PRECISION)];
delta0_interval = [pc(u(2)-delta_UNCERTAINTY, PRECISION), pc(u(2)+delta_UNCERTAINTY, PRECISION)];


initial_state_str_linear = num2str(x0_interval(1))+ " " +num2str(x0_interval(2)) ...
    + " " + num2str(y0_interval(1))+ " " +num2str(y0_interval(2)) ...
    + " " + num2str(phi0_interval(1))+ " " +num2str(phi0_interval(2)) ...
    + " " + num2str(v0_interval(1))+ " " +num2str(v0_interval(2)) ...
    + " " + num2str(delta0_interval(1))+" "+num2str(delta0_interval(2)) ...
    + " " + num2str(v_noise_min)+" "+num2str(v_noise_max) + " " + num2str(delta_noise_min) + " "+num2str(delta_noise_max);

if strcmp(model, 'linear')
    command = "cd linear_model;./RC_bicycle "+parameter_str_linear+" "+initial_state_str_linear;
    [status,cmdout] = system(command);
    [x_flow, y_flow] = extract_flowpipes(0.02, 0.1, model);
else
    command = "cd nonlinear_model;./RC_bicycle "+" "+initial_state_str_linear;
    [status,cmdout] = system(command);
    [x_flow, y_flow] = extract_flowpipes(0.1, 0.1, model);        
end
x_flow = x_flow+x0;
y_flow = y_flow+y0;
end
