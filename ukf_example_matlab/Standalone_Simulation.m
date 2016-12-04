clear var
clc

% X: x, theta1, theta2, dot_x, dot_theta1, dot_theta2 
x_init = [0.0, 0.001, 0.0, 0.0, 0.0, 0.0]';

tspan = [0:0.001:10]';

[t,y] = ode45(@SystemDerivatives,tspan,x_init);


% Display

figure(1);

subplot(3,2,1)
plot(t,y(:,1))
xlabel('Time (sec)')
ylabel('Cart Position (m)')

subplot(3,2,2)
plot(t,y(:,4))
xlabel('Time (sec)')
ylabel('Cart Velocity (m/s)')

subplot(3,2,3)
plot(t,rad2deg(wrapToPi(y(:,2))))
xlabel('Time (sec)')
ylabel('Pedulum 1 angle (deg)')

subplot(3,2,4)
plot(t,rad2deg(y(:,5)))
xlabel('Time (sec)')
ylabel('Pedulum 1 angular velocity (deg/s)')

subplot(3,2,5)
plot(t,rad2deg(wrapToPi(y(:,3))))
xlabel('Time (sec)')
ylabel('Pedulum 2 angle (deg)')

subplot(3,2,6)
plot(t,rad2deg(y(:,6)))
xlabel('Time (sec)')
ylabel('Pedulum 2 angular velocity (deg/s)')