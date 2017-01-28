figure(1);

%Points = length(command)-35:length(command)-30;
 Points = 1:length(command);

subplot(3,2,1)
plot(state(1,Points))
xlabel('Time (sec)')
ylabel('Cart Position (m)')

subplot(3,2,2)
plot(state(4,Points))
xlabel('Time (sec)')
ylabel('Cart Velocity (m/s)')

subplot(3,2,3)
plot(rad2deg(wrapToPi(state(2,Points))))
xlabel('Time (sec)')
ylabel('Pedulum 1 angle (deg)')

subplot(3,2,4)
plot(rad2deg(state(5,Points)))
xlabel('Time (sec)')
ylabel('Pedulum 1 angular velocity (deg/s)')

subplot(3,2,5)
plot(rad2deg(wrapToPi(state(3,Points))))
xlabel('Time (sec)')
ylabel('Pedulum 2 angle (deg)')

subplot(3,2,6)
plot(rad2deg(state(6,Points)))
xlabel('Time (sec)')
ylabel('Pedulum 2 angular velocity (deg/s)')

figure(2);
plot(command(Points))
xlabel('Time (sec)')
ylabel('Command (N)')
