function response = serviceCallback(server,reqmsg,defaultrespmsg)

format long

response = defaultrespmsg;
% Build the response message here

% System parameters

m0 = 1.5;  % kg
m1 = 0.5;  % kg
m2 = 0.75; % kg
L1 = 0.5;  % m
L2 = 0.75; % m

Q  = diag([5 50 50 20 700 700]);
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

x = [reqmsg.X, reqmsg.Theta, reqmsg.Phi, reqmsg.DotX, reqmsg.DotTheta, reqmsg.DotPhi]';

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

H = [1 0 0]';

% SDRE controler
if 0
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

ts = 0.02; % s

AA = expm(A*ts);
BB = B*ts;

u = 0;

try
    [~,~,K] = dare(AA,BB,Q,R);
    u = -1*K*x;
catch
    rosshutdown
end

response.Force = u;

end
