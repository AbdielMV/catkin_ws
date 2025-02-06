% clear all
% clc
close all


% Load the data
data_robot = readtable('robot_states.csv');
data_rhonn = readtable('rhonn_states.csv');
% data_control = readtable('control_states.csv');

% Extract columns
time_robot = data_robot.Time;
position_robot = data_robot.Position;
velocity_robot = data_robot.Velocity;
effort_robot = data_robot.Effort;

% Extract columns
time_rhonn = data_rhonn.Time;
position_rhonn = data_rhonn.Position;
velocity_rhonn = data_rhonn.Velocity;
error_xn1 = data_rhonn.ErrorP;
error_xn2 = data_rhonn.ErrorV;

%Extract columns
time_control = data_rhonn.Time;
error_z0 = data_rhonn.Z0;
error_z1 = data_rhonn.Z1;
ref_control = data_rhonn.Reference;

% Plot the data Robot
figure;
subplot(2,1,1);
plot(time_robot, rad2deg(position_robot));
xlabel('Time (s)');
ylabel('Position');
title('Position vs Time');

subplot(2,1,2);
plot(time_robot, rad2deg(velocity_robot));
xlabel('Time (s)');
ylabel('Velocity');
title('Velocity vs Time');

%Plot the data Control Law
figure;
plot(time_robot, effort_robot);
xlabel('Time (s)');
ylabel('Effort');
title('Effort vs Time');

% Plot the data Rhonn
figure;
subplot(2,1,1);
plot(time_rhonn, rad2deg(position_rhonn));
xlabel('Time (s)');
ylabel('Position');
title('Position vs Time');


subplot(2,1,2);
plot(time_rhonn, rad2deg(velocity_rhonn));
xlabel('Time (s)');
ylabel('Velocity');
title('Velocity vs Time');

%Plot data error
figure
subplot(4,1,1);
plot(time_rhonn,rad2deg(error_xn1));
xlabel('Time (s)');
ylabel('Degree');
title('Error Position Ident');

subplot(4,1,2);
plot(time_rhonn,rad2deg(error_xn2));
xlabel('Time (s)');
ylabel('Error');
title('Error Velocity Ident');

subplot(4,1,3);
plot(time_control,rad2deg(error_z0));
xlabel('Time (s)');
ylabel('Error');
title('Error Tracking Z0');

subplot(4,1,4);
plot(time_control,rad2deg(error_z1));
xlabel('Time (s)');
ylabel('Error');
title('Error Tracking Z1');

%Plot the data With All
figure
subplot(2,1,1);
plot(time_robot,rad2deg(position_robot));
hold on
plot(time_rhonn,rad2deg(position_rhonn));
plot(time_control,rad2deg(ref_control));
plot(time(1,(1:end-1)),rad2deg(X(1,:)));
plot(time,rad2deg(y(1,1:end-1)));
plot(time,rad2deg(XN(1,:)));
hold off
xlabel('Time (s)');
ylabel('Position');
title('Position vs Time');
legend('Motor ROS','Rhonn ROS','Ref ROS','Motor MATLAB','Ref MATLAB','Rhonn MATLAB');

subplot(2,1,2);
plot(time_robot,rad2deg(velocity_robot));
hold on
plot(time_rhonn,rad2deg(velocity_rhonn));
plot(time(1,(1:end-1)),rad2deg(X(2,:)));
plot(time,rad2deg(XN(2,:)));
hold off
xlabel('Time (s)');
ylabel('Velocity');
title('Velocity vs Time');
legend('Velocity ROS','Rhonn ROS','Velocity MATLAB','Rhonn MATLAB')

figure
subplot(2,1,1);
plot(time_rhonn,rad2deg(error_xn1));
hold on
plot(time(1:end-1),rad2deg(error(1,:)));
hold off
xlabel('Time (s)');
ylabel('Degree');
title('Error Position Ident');
legend('Error ROS','Error MATLAB')

subplot(2,1,2);
plot(time_rhonn,rad2deg(error_xn2));
hold on
plot(time(1:end-1),rad2deg(error(2,:)));
xlabel('Time (s)');
ylabel('Error');
title('Error Velocity Ident');
legend('Error ROS','Error MATLAB')

figure
subplot(2,1,1);
plot(time_control,rad2deg(error_z0));
hold on
plot(time(1,(1:end)),rad2deg(z0));
hold off
xlabel('Time (s)');
ylabel('Error');
title('Error Tracking Z0');
legend('Error Z0 ROS','Error Z0 MATLAB')

subplot(2,1,2);
plot(time_control,rad2deg(error_z1));
hold on
plot(time(1,(1:end)),rad2deg(z1));
hold off
xlabel('Time (s)');
ylabel('Error');
title('Error Tracking Z1');
legend('Error Z1 ROS','Error Z1 MATLAB')

figure
plot(time_robot, effort_robot);
hold on
plot(time,u);
xlabel('Time (s)');
ylabel('Effort');
title('Effort vs Time');
legend('Torque ROS','Torque MATLAB')