function lqr_params = get_LQR(x)

n = length(x);
lqr_params.A = zeros(n,n);
lqr_params.B = eye(n);
lqr_params.C = eye(n);
lqr_params.D = 0;

lqr_params.Q = 10*eye(n);
lqr_params.R = 100*eye(n);
lqr_params.K = lqr(lqr_params.A, lqr_params.B, lqr_params.Q, lqr_params.R);

end