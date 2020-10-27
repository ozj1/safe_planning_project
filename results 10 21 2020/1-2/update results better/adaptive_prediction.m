function [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H_old, VAR_old, step)
f = @(A,x) [A(1,1)*x(1)+A(1,2)*x(2)+A(1,3)*x(3);
            A(2,1)*x(1)+A(2,2)*x(2)+A(2,3)*x(3);
            A(3,1)*x(1)+A(3,2)*x(2)+A(3,3)*x(3)];
dim = 3;
noise = 0;
h = 1;
ADAPT = 1;
LINEAR = 1;
lambda = 0;
sampling_time = 0.1;
time_step = 0.01;%time step for flow*
C = 3; %uncertainty plot range
PRECISION = 4;

H_short = 0.5;
H_long = 4;
x_3d = [x(1,:);x(2,:);x(5,:)];%x, y, yaw
x_3d_old = [x_old(1,:);x_old(2,:);x_old(5,:)];
%T = length(X)-1; % Length of whole trajectory
u = [x(4,:);0];
%X0 = X; % Initial state
%H = ones(T+1,1);

%x = X;

xhat = predict(f, ahat_old, x_3d_old, h);%last prediction, grund truth current value
parameter_str = "";
for i = 1: dim
    if ADAPT
        H = lambda*H_old; % inverse of learning gain
        e = x_3d(i) - xhat(i, end);
        G = x_3d_old;
        H = H + G'*G;
        ahat(i, :) = ahat_old(i, :) + inv(H)*G'*e;
        for j = 1:dim
            ahat(i, j) = pc(ahat(i, j), PRECISION);
            if i == j
                ahat_continu(i, j) = (1/sampling_time)*(ahat(i, j)-1); 
            else
                ahat_continu(i, j) = (1/sampling_time)*ahat(i, j);
            end
        end
        F = inv(H);
        VAR(i, :) = VAR_old(i, :)*(1-F*x_3d_old(i, :)^2)^2 + (F*x_3d_old(i, :))^2*noise^2; %Update variance
        %VAR(i, t) = min(VAR(i, t), 0.01);
        BOUND(i, :) = C*sqrt(VAR(i, :));
        for j = 1:dim
            a_interval{i, j} = [ahat_continu(i, j)-BOUND(i, :), ahat_continu(i, j)+BOUND(i, :)];
        end
    end
end

xhat11 = predict(f, ahat, x_3d, step);
% size(xhat11);
if step == 40
    x_long_term = xhat11;
end
if step == 10
    x_long_term = xhat11(:, end);
end
if step == 4
    x_long_term = xhat11(:, [2, 3, 4, 5]);
end
end