function [x_hat,P_hat] = update(x_prd, P_prd, y, u, Cd, Dd, Rd, n)
    K = P_prd * Cd' * inv(Cd * P_prd * Cd' + Rd);
    IKC = eye(n) - K * Cd;
    x_hat = x_prd + K * ssa(y - Cd * x_prd - Dd * u);
    P_hat = IKC * P_prd * IKC' + K * Rd * K';
end