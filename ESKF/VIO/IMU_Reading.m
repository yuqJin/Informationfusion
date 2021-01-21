% 模拟IMU的数据读取
% 2021/1/20 14:34
% By 金宇强

%% 使用waypoint模拟轨迹
% Sampling rate
Fs = 100;

% Waypoints and times of arrival
waypoints = [1 1 1; 3 1 1; 3 0 0; 0 0 0];
t = [1; 10; 20; 30];

% Create trajectory and compute pose
traj = waypointTrajectory(waypoints, t, "SampleRate", Fs);
% 位置；方向（四元数）；速度；加速度；角速度；
[posVeh, orientVeh, velVeh, accVeh, angvelVeh]= lookupPose(traj, t(1):1/Fs:t(end));


%% IMU模拟并定义偏移量
%IMU at vehicle origin
imu = imuSensor("accel-gyro-mag", "SampleRate",Fs);

%Position and orientation offset of vehicle and mounted IMU
posVeh2IMU=[2.4 0.5 0.4];
orientVeh2IMU=quaternion([0 0 90],"eulerd","ZYX","frame");

%Visualization
helperPlotIMU(posVeh(1,:), orientVeh(1,:), posVeh2IMU, orientVeh2IMU);

%% 使用汽车轨迹计算IMU轨迹
%使用transformMotion函数计算安装在驾驶员座椅上的IMU的地面真实轨迹。
%该功能使用位置和方向偏移以及车辆轨迹来计算IMU轨迹。
[posIMU, orientIMU, velIMU, accIMU, angvelIMU] = transformMotion( ...
    posVeh2IMU, orientVeh2IMU, ...
    posVeh, orientVeh, velVeh, accVeh, angvelVeh);

%% 生成IMU读数
% 为安装在车辆原点的IMU生成IMU读数。
% IMU at vehicle origin.
[accel, gyro, mag] = imu(accVeh, angvelVeh, orientVeh);


%% 比较两个IMU的加速度计读数
figure('Name', 'Accelerometer Comparison')
subplot(3, 1, 1)
plot([accel(:,1),accVeh(:,1)])
legend('Accele', 'Vel')
title('Accelerometer')
ylabel('x-axis (m/s^2)')
subplot(3, 1, 2)
plot([accel(:,2), accVeh(:,2)])
ylabel('y-axis (m/s^2)')
subplot(3, 1, 3)
plot([accel(:,3), accVeh(:,3)])
ylabel('z-axis (m/s^2)')