clc
clear
%% 
load cut_in_traj
X = cut_in_traj;
% load cut_in_traj
f = @(A,x) [A(1,1)*x(1)+A(1,2)*x(2)+A(1,3)*x(3);
            A(2,1)*x(1)+A(2,2)*x(2)+A(2,3)*x(3);
            A(3,1)*x(1)+A(3,2)*x(2)+A(3,3)*x(3)];
ahat = [];
for i=1:120
    [x_flow, y_flow] = reach(X(:, i));
%%%%%%%%%%%%%%%%%%%%%%%0.1 second update version%%%%%%%%%%%%%%%%%
    if i == 1
        continue;
    elseif i == 2
        x_old = X(:,1);
        x = X(:,2);
        ahat_old = diag(ones(1, 3));
        H = 1;
        VAR_old = 0.001*ones(3, 1);%0.0001
        BOUND = ones(3, 1);
        [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H, VAR_old, 40);
    else
        x_old = X(:, i-1);
        x = X(:, i);
        ahat_old = ahat;
        VAR_old = VAR;
        H_old = H;   
        %ahat
        [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H, VAR_old, 40);
    end
    
    % linear update
    x_linear = X(:,i) * ones(1,41);
    for j = 2:(41)
       x_linear(1,j) = x_linear(1,j-1) + 0.1*x_linear(3,j)*cos(x_linear(4,j));
       x_linear(2,j) = x_linear(2,j-1) + 0.1*x_linear(3,j)*sin(x_linear(4,j));
    end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%1 second update%%%%%%%%%%%%%%%%%
%     if i == 1
%         continue;
%     elseif i == 2
% %     elseif i < 12
%         x_old = X(:,1);
%         x = X(:,2);
%         ahat_old = diag(ones(1, 3));
%         H = 1;
%         VAR_old = 0.001*ones(3, 1);%0.0001
%         BOUND = ones(3, 1);
%         [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H, VAR_old, 40);
%         %ahat
%     elseif i> 2 && i < 12
%         xhat = predict(f, ahat, [X(1:2,i); X(4,i)], 40);
%         x_long_term = xhat(:, [1, 11, 21, 31, 41]);
%     elseif i > 2 && mod(i-2, 10)==0
% %     elseif i >= 12
%         x_old = x;
%         x = X(:, i);
%         ahat_old = ahat;
%         VAR_old = VAR;
%         H_old = H;   
%         %ahat
%         [x_long_term, ahat, H, VAR] = adaptive_prediction(x, x_old, ahat_old, H, VAR_old, 4);
%     else
%         xhat = predict(f, ahat, [X(1:2,i); X(4,i)], 4);
%         x_long_term = xhat11(:, [2, 3, 4, 5]);
%     end
    
%     fprintf('i = %d', i);
%     x_long_term
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(1)
    plot(X(1, :), X(2, :), 'b-')
    hold on
    plot(x_long_term(1,:), x_long_term(2,:), 'r*')
    hold on
    plot(x_flow, y_flow, 'g')
    hold on
    plot(x_linear(1,:), x_linear(2,:), 'c*')
    hold off
    pause(0.1)
end
