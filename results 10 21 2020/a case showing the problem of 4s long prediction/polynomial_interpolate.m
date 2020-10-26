function x_fitted = polynomial_interpolate(x0, x, dt, ts)
% polynomial__interpolate: interpolates between given trajectory
%
% inputs
% ======
% x0:   initial state 
% x:    input trajectory
% dt:   input trajectory discretization
% ts:   desired sampling time
%
% output
% ======
% x_fitted: interpolated trajectory

N       = size(x, 2);
x_dim   = 4;
k       = dt/ts;

% 3rd order polynomial fitting matrix
T_mat = @(x0, xf) [1, x0, x0^2, x0^3;
                   0, 1,  2*x0, 3*x0^2;
                   1, xf, xf^2, xf^3;
                   0, 1,  2*xf, 3*xf^2];
% initialization
x_fitted = zeros(x_dim, N*k);
x_fitted(:,1) = x0;

% looping through vertices
for i = 1:N
    if i == 1
        q  = [x0(2); tan(x0(4)); x(2,i); tan(x(3,i))];
        a = T_mat(x0(1), x(1,i)) \  q;
    else
        q  = [x(2,i-1); tan(x(3,i-1)); x(2,i); tan(x(3,i))]; 
        a = T_mat(x(1,i-1), x(1,i)) \ q;
    end
    y  = @(x) a' * [1; x; x^2; x^3];
    dy = @(x) a' * [0; 1; 2*x; 3*x^2];
    for j = 1:(dt/ts)
        idx     = (i-1)*k + j;
        if (i == 1)
            x_t     = x0(1) + (x(1,i) - x0(1))/k*j;
        else
            x_t     = x(1,i-1) + (x(1,i) - x(1,i-1))/k*j;
        end        
        y_t     = y(x_t);
        
        if (idx == 1)
            continue;
        end
        v_t     = sqrt((x_t - x_fitted(1, idx-1))^2 + ...
                    (y_t - x_fitted(2, idx-1))^2)/ts;        
        theta_t = atan(dy(x_t));
        x_fitted(:, idx) = [x_t; y_t; v_t; theta_t];
    end
end


% for i = 1:N
%     if i == 1
%         vt = sqrt((x(1,i) - x0(1))^2 + ...
%                     (x(2,i) - x0(2))^2)/dt;
%         qx = [x0(1), x0(3)*sin(x0(4)), x(1,i), vt*sin(x(3,i))];
%         qy = [x0(2), x0(3)*sin(x0(4)), x(2,i), vt*sin(x(3,i))];
%     else
%         vt = sqrt((x(1,i) - x(1,i-1))^2 + ...
%                     (x(2,i) - x(2,i-1))^2)/dt;
%         qx = [x(1,i-1), v_pre*cos(x(3, i-1)), x(1,i), vt*cos(x(3,i))];
%         qy = [x(2,i-1), v_pre*sin(x(3, i-1)), x(2,i), vt*sin(x(3,i))];
%         v_pre = vt;
%     end
%     a      = inv(T_mat(0, dt))*qx';... 4x1
%     b      = inv(T_mat(0, dt))*qy';... 4x1
%     t_now  = @(t) [1, t, t^2, t^3]';
%     dt_now = @(t) [0, 1, 2*t, 3*t^2]';
%     
%     % x, y
%     x_now  = @(t) a'*t_now(t);
%     y_now  = @(t) b'*t_now(t);
%     
%     % dy/dx
%     dx_now  = @(t) a'*dt_now(t);
%     dy_now  = @(t) b'*dt_now(t);
%     
%     for j = 1:(dt/ts)
%         idx     = (i-1)*k + j;
%         if (idx ~= 1)
%             x_t     = x_now(j*ts);
%             y_t     = y_now(j*ts);
%             v_t     = sqrt((x_t - x_fitted(1, idx-1))^2 + ...
%                             (y_t - x_fitted(2, idx-1))^2)/ts;
%             theta_t = atan(dy_now(j*ts)/dx_now(j*ts));
%             x_fitted(:, idx) = [x_t; y_t; v_t; theta_t];
%         else
%             theta_t = atan(dy_now(j*ts)/dx_now(j*ts));
%             x_fitted(4, idx) = theta_t;
%         end
%     end
% end


end