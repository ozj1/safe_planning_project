function [X_new, U_new] = forward_pass(U, X, k, K, x_start, Ts, Alpha)
% forward_pass: roll out trajectory based on backward_pass result
%
% input
% =====
% U:        last iteration control sequence
% X:        last iteration planned trajectory
% k:        control gain calculated by backward_pass
% K:        state gain calculated by backward_pass
% x_start:  initial state for the horizon
% Ts:       sampling time
% Alpha:    line search parameter
%
% output
% ======
% X_new:    new planned trajectory
% U_new:    new planned control sequence

global NUM_CTRL;
global X_DIM U_DIM;
global L;
global a_dot_max a_dot_min v_dot_max v_dot_min delta_max delta_min

X_new = zeros(X_DIM, NUM_CTRL);
U_new = zeros(U_DIM, NUM_CTRL);

X_new(:, 1) = x_start;
isempty = ~(size(k, 1));    ... if k is empty, then perform kinematics update
for i = 1 : NUM_CTRL
    if ~isempty
        % update control sequence
        du = Alpha * k(:, i) + K(:, :, i) * (X_new(:, i) - X(:, i));
        U_new(:, i) = U(:, i) + du;

        % define upper and lower bound for controller
        U_ub = [a_dot_max; delta_max];
        U_lb = [a_dot_min; delta_min];

%         disp(Alpha * k(:, i));
%                 disp( (X_new(:, i) - X(:, i)));
% disp( K(:, :, i) * (X_new(:, i) - X(:, i)));
        % clip control usage if over the bound
        U_new(1,i) = min(max(U_lb(1),U_new(1,i)), U_ub(1));
        U_new(2,i) = min(max(U_lb(2),U_new(2,i)), U_ub(2));

        % use discrete dynamics to update the states        
        x0      = X_new(1, i);
        y0      = X_new(2, i);
        a0      = X_new(3, i);
        v0      = X_new(4, i);
        theta   = X_new(5, i);
        jerk     = U_new(1,i);
        delta   = U_new(2,i);
    else
        x0      = X_new(1, i);
        y0      = X_new(2, i);
        a0      = X_new(3, i);
        v0      = X_new(4, i);
        theta   = X_new(5, i);
        jerk     = U(1,i);
        delta   = U(2,i);
    end
    % kinematics update
    curvature = tan(delta)/L;
    l = v0 * Ts + 0.5 * a0 * Ts^2+ (1/6.) * jerk * Ts^3;         ... distance traveled
    X_new(4, i+1) = v0 + a0 * Ts+0.5 * jerk * Ts^2;          ... velocity update
    X_new(3, i+1) = a0 + jerk * Ts;          ... acc update

    X_new(5, i+1) = theta + curvature * l;  ... heading update
    
    if (curvature == 0)
        X_new(1, i+1) = x0 + l * cos(theta);
        X_new(2, i+1) = y0 + l * sin(theta); 
    else
        X_new(1, i+1) = x0 + (sin(theta + curvature * l) - ...
            sin(theta))/curvature;
        X_new(2, i+1) = y0 + (cos(theta) - cos(theta + ...
            curvature * l))/curvature;
    end
end

end