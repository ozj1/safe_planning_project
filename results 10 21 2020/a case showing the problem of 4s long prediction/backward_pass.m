function [k, K] = backward_pass(cx,cu,cxx,cux,cuu,fx,fu,lambda)
% forward_pass: roll out trajectory based on backward_pass result
%
% input
% =====
% cx:       first derivative of cost fcn w.r.t state X
% cu:       first derivative of cost fcn w.r.t control u
% cxx:      second derivative of cost fcn w.r.t state X
% cux:      second derivative of cost fcn w.r.t control u and state X
% cuu:      second derivative of cost fcn w.r.t control u
% fx:       first derivative of car dynamics w.r.t state X
% fu:       first derivative of car dynamics w.r.t state u
% lambda:   Levenberg - Marquardt heuristics implementation
%
% all derivatives are matrices
%
% output
% ======
% diverge:  whether the iteration has diverged
% Vx:       first derivative of cost-to-go function w.r.t state X  
% Vxx:      second derivative of cost-to-go function w.r.t state X
% k:        calculated control gain
% K:        calculated state gain

global X_DIM U_DIM;
global NUM_CTRL;

n = X_DIM;
m = U_DIM;
N = NUM_CTRL + 1;

k   = zeros(m,N-1);
K   = zeros(m,n,N-1);

Vx  = cx(:,N);
Vxx = cxx(:,:,N);

for i = N-1:-1:1
    Qu  = cu(:,i)      + fu(:,:,i)'*Vx;
    Qx  = cx(:,i)      + fx(:,:,i)'*Vx;
    
    Qux = cux(:,:,i)   + fu(:,:,i)'*Vxx*fx(:,:,i);
    Quu = cuu(:,:,i)   + fu(:,:,i)'*Vxx*fu(:,:,i);   
    Qxx = cxx(:,:,i)   + fx(:,:,i)'*Vxx*fx(:,:,i);
    
    % calculate Quu_inv with inv()
    Quu_inv = inv(Quu);

%     % calculate Quu_inv with eigen decomp and make sure Quu is PD
%     [Quu_evecs, Quu_evals_mat] = eig(Quu);
%     Quu_evals = diag(Quu_evals_mat);
%     Quu_evals(Quu_evals < 0) = 0;
%     Quu_evals = Quu_evals + lambda;
%     Quu_inv = Quu_evecs * diag(1./Quu_evals) * inv(Quu_evecs);
    
    % update k and K
    k(:,i)      = -Quu_inv * Qu;    % control gain
    K(:,:,i)    = -Quu_inv * Qux;   % state gain
%     k(:,i)      = sym(-Quu, 'f')\sym(Qu, 'f');    % control gain added by
%     %Omid I commented this to tak care of this error: Warning: Matrix is close to singular or badly scaled. Results may be inaccurate. RCOND = 7.673400e-265. 
%     K(:,:,i)    =sym(-Quu, 'f')\sym(Qux, 'f');   % state gain
    
    % update Value function Vclc
    Vx      = Qx + transpose(K(:,:,i)) * Quu *k(:,i); %4*1
    %size(Vx)
    %size(Qx)
    Vxx     = Qxx + transpose(K(:,:,i)) * Quu * K(:,:,i); %4*4
    %size(Vxx)
end

end