m = 180;
R_33 = 2;
k_p = 2;
k_d = 40;

A =[zeros(3), 1/2*eye(3);
    zeros(3,6)];
B = [zeros(3); 1/(m*R_33^2)*eye(3)];

K = [k_p*eye(3); k_d*eye(3)]';

lambda = eig(A-B*K);

