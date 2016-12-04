function [dot_x] = SystemDerivatives(~,x)
format long

u =0;

% System parameters

m0 = 1.5;  % kg
m1 = 0.5;  % kg
m2 = 0.75; % kg
L1 = 0.5;  % m
L2 = 0.75; % m

drag1 = 0.005; % N m s
drag2 = 0.005; % N m s

ts = 0.02; % s

% l1 = L1/2; % m
% l2 = L2/2; % m
% I1 = m1*L1^2/12; % N m s²
% I2 = m2*L2^2/12; % N m s²

Q  = diag([5 50 50 20 700 700]);
Q  = diag([1 1 1 1 1 1]);
R  = 1;

g = 9.81;

% Compupted coefficients

d1 = m0 + m1 + m2;
d2 = (0.5 * m1 + m2) * L1;
d3 = 0.5 * m2 * L2;
d4 = (1/3 * m1 + m2) * L1^2;
d5 = 0.5 * m2 * L1 * L2;
d6 = 1/3 * m2 * L2^2;
f1 = (0.5 * m1 + m2) * L1 * g;
f2 = 0.5 * m2 * L2 * g;

% System dynamics

theta1      = x(2);
theta2      = x(3);
dot_theta1  = x(5);
dot_theta2  = x(6);

D = [d1,              d2*cos(theta1),         d3*cos(theta2);
    d2*cos(theta1),  d4,                     d5*cos(theta1-theta2);
    d3*cos(theta2),  d5*cos(theta1-theta2)   d6];

C = [0, -d2*sin(theta1)*dot_theta1,        -d3*sin(theta2)*dot_theta2;
    0,  0,                                 d5*sin(theta1-theta2)*dot_theta2;
    0  -d5*sin(theta1-theta2)*dot_theta1,  0];

G = [0, -f1*sin(theta1), -f2*sin(theta2)]';

H = [1 0 0]';

F = [0 -drag1*dot_theta1+drag2*(dot_theta2-dot_theta1) drag2*(dot_theta1-dot_theta2)]';

matrix_1 = [zeros(3), eye(3);
    zeros(3), -D\C];
matrix_2 = [zeros(3,1); -D\G];

matrix_3 = [zeros(3,1); D\H];

matrix_4 = [zeros(3,1); D\F];

% SDRE controler
if 1
    G = zeros(3);
    if(deg2rad(0.0001) < abs(theta1))
        G(2,2) = -f1 * sin(theta1)/theta1;
    end
    
    if(deg2rad(0.0001) < abs(theta2))
        G(3,3) = -f2 * sin(theta2)/theta2;
    end
else
    %LQR
    G = diag([0, -f1*cos(theta1), -f2*cos(theta2)]);

end


A = [ zeros(3), eye(3);
    -D\G,  -D\C];

B = [zeros(3,1);
    D\H];

[P,L,G] = care(A,B,Q,R);

u = -R\B'*P*x;

dot_x = matrix_1 * x + matrix_2 + matrix_3 * u + matrix_4;

