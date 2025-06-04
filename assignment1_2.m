tau = [0 1 2 3 4 5];
sigma_lower = [];
sigma_upper = [];

A  = [
    0 1 1 0 1;
    1 0 1 0 1;
    1 1 0 1 0;
    0 0 1 0 1;
    1 1 0 1 0;
];

D=diag(sum(A,2));

L = D - A;

eigen = eig(L);
lambda_2 = eigen(3);
lambda_N = eigen(end);

S_2_lower = lamdba_2./2.*sigma_lower;
S_2_upper = lamdba_2./2.*sigma_upper;
S_N_lower = lamdba_N./2.*sigma_lower;
S_N_upper = lamdba_N./2.*sigma_upper;

