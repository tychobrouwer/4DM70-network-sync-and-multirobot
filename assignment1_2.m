tau = [0 1 2 3 4 5];
sigma_lower = [0.49 0.55 0.60 0.69 0.75 0.81];
sigma_upper = [10.0  8.7  5.2  4.1  3.7 3.4];

plot(sigma_upper, tau, 'Color', 'blue', 'LineWidth', 3); hold on;
plot(sigma_lower, tau, 'Color', 'blue', 'LineWidth', 3);

A  = [
    0 1 1 0 1;
    1 0 1 0 1;
    1 1 0 1 0;
    0 0 1 0 1;
    1 1 0 1 0
];

D=diag(sum(A,2));
L = D - A;

% eigen = eig(L);
syms lambda
I = eye(size(L));
char_poly = det(A - lambda*I);
eigen = sort(double(solve(char_poly, lambda)));

lambda_2 = eigen(4);
lambda_N = eigen(end);

S_2_lower = lambda_2./2.*sigma_lower;
S_2_upper = lambda_2./2.*sigma_upper;
S_N_lower = lambda_N./2.*sigma_lower;
S_N_upper = lambda_N./2.*sigma_upper;

plot(S_2_lower, tau, 'Color', 'red', 'LineWidth', 3);
plot(S_2_upper, tau, 'Color', 'red', 'LineWidth', 3);
plot(S_N_lower, tau, 'Color', 'cyan', 'LineWidth', 3);
plot(S_N_upper, tau, 'Color', 'cyan', 'LineWidth', 3);
xlim([0 10]);
ylim([0 5.5]);

sigma_lower_measured = [0.55 0.60 0.68 0.77 0.86 0.95];
sigma_upper_measured = [  10 3.94 2.5  2.2 1.70 1.30];

A1  = [
    0 1 1 1 1;
    1 0 1 0 1;
    1 1 0 1 0;
    1 0 1 0 1;
    1 1 0 1 0
]; % added to 1-4
D1 = diag(sum(A1,2));
L1 = D1 - A1;

char_poly = det(L1 - lambda*I);
eigen1 = sort(double(solve(char_poly, lambda)));

A2  = [
    0 1 1 0 1;
    1 0 1 0 1;
    1 1 0 1 1;
    0 0 1 0 1;
    1 1 1 1 0
]; % added to 3-5
D2 = diag(sum(A2,2));
L2 = D2 - A2;

char_poly = det(L2 - lambda*I);
eigen2 = sort(double(solve(char_poly, lambda)));

bigger_connectivity = eigen1(2) > eigen2(2); 

sigma_lower_1_tau_5 = 0.70;
sigma_upper_1_tau_5 = 1.4;

