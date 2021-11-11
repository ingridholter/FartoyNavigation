function [x_prd,P_prd] = predict(x_hat, P_hat, u, Ad, Bd, Ed, Qd)
    x_prd = Ad * x_hat + Bd * u;
    P_prd = Ad * P_hat * Ad' + Ed * Qd * Ed';
end