%% 2021/1/20 
% 创建一个包含车辆地面真实轨迹的驾驶场景。
% 使用IMU和视觉里程表模型生成测量值。
% 融合这些测量值以估计车辆的姿态，然后显示结果。

clear
%% 创建具有轨迹的驾驶场景
% 创建一个包含 道路/建筑物/车辆的地面真实姿势/车辆的估计姿势
% 车辆的地面真实姿态显示为蓝色长方体，估计的姿势显示为透明的蓝色长方体。
% 使用waypointTrajectory生成地面车辆的基线轨迹
scene = drivingScenario;
% groundtruth 真值
groundTruthVehicle = vehicle(scene, 'PlotColor', [0 0.4470 0.7410]);
% 估计值
estVehicle = vehicle(scene, 'PlotColor', [0 0.4470 0.7410]);

% Generate the baseline trajectory.
sampleRate = 100;
wayPoints = [  0   0 0;
             200   0 0;
             200  50 0;
             200 230 0;
             215 245 0;
             260 245 0;
             290 240 0;
             310 258 0;
             290 275 0;
             260 260 0;
             -20 260 0];
t = [0 20 25 44 46 50 54 56 59 63 90].';
speed = 10;
velocities = [ speed     0 0;
               speed     0 0;
                   0 speed 0;
                   0 speed 0;
               speed     0 0;
               speed     0 0;
               speed     0 0;
                   0 speed 0;
              -speed     0 0;
              -speed     0 0;
              -speed     0 0];

traj = waypointTrajectory(wayPoints, 'TimeOfArrival', t, ...
    'Velocities', velocities, 'SampleRate', sampleRate);

% Add a road and buildings to scene and visualize.
helperPopulateScene(scene, groundTruthVehicle);


%% 创建一个融合过滤器
% 使用松耦合的方法融合测量结果。尽管结果不如紧密耦合方法那样准确。融合滤波器
% 使用误差状态卡尔曼滤波来跟踪方向，位置，速度和传感器偏差
% 该insfilterErrorState对象具有以下功能来处理传感器数据：predict和fusemvo。
% 该predict功能将IMU的加速度计和陀螺仪测量结果作为输入。
% predict每次对加速度计和陀螺仪采样时都调用该函数。
% 该功能基于加速度计和陀螺仪的测量值，以一个时间步长预测状态，
% 并更新滤波器的误差状态协方差。
filt = insfilterErrorState('IMUSampleRate', sampleRate, ...
    'ReferenceFrame', 'ENU');
% Set the initial state and error state covariance.
helperInitialize(filt, traj);

%% 视觉里程表模型
% 定义视觉里程表模型参数。这些参数通过单目相机对特征匹配和基于跟踪的
% 视觉视觉里程计系统建模。
% 尺度参数说明了单目相机后续视觉框架的未知比例。
% 其他参数将视觉里程计读数中的偏差建模为白噪声和一阶马尔可夫模型
% flag useVO 表示VO是否使用
% useVO = false; % Only IMU is used.
useVO = true; % Both IMU and visual odometry are used.

% VO的相关参数
paramsVO.scale = 2;
paramsVO.sigmaN = 0.139;
paramsVO.tau = 232;
paramsVO.sigmaB = sqrt(1.34);
paramsVO.driftBias = [0 0 0];

%% 惯导模型
% 使用imuSensorSystem对象定义包含加速度计和陀螺仪的IMU传感器模型。模型包
% 含对确定性和随机噪声源进行建模的属性。此处的属性值是低成本MEMS传感器的典型值。
% Set the RNG seed to default to obtain the same results for subsequent
% runs.
rng('default')

imu = imuSensor('SampleRate', sampleRate, 'ReferenceFrame', 'ENU');

% Accelerometer
imu.Accelerometer.MeasurementRange =  19.6; % m/s^2
imu.Accelerometer.Resolution = 0.0024; % m/s^2/LSB
imu.Accelerometer.NoiseDensity = 0.01; % (m/s^2)/sqrt(Hz)

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250); % rad/s
imu.Gyroscope.Resolution = deg2rad(0.0625); % rad/s/LSB
imu.Gyroscope.NoiseDensity = deg2rad(0.0573); % (rad/s)/sqrt(Hz)
imu.Gyroscope.ConstantBias = deg2rad(2); % rad/s

%% 仿真设置
% 指定仿真时间并初始化在仿真循环中记录的变量
% 仿真时间60s
numSecondsToSimulate = 80;
numIMUSamples = numSecondsToSimulate * sampleRate;

% VO的采样率
imuSamplesPerCamera = 4;
numCameraSamples = ceil(numIMUSamples / imuSamplesPerCamera);

% 预分配数据数组以绘制结果
[pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst] ...
    = helperPreallocateData(numIMUSamples, numCameraSamples);

% 为VO设置测量噪声参数
RposVO = 0.1;
RorientVO = 0.1;

%% 运行仿真循环
% 以IMU采样率运行仿真。每个IMU样本都用于通过一个时间步向前预测滤波器的状态。
% 一旦有新的视觉里程计读数可用，它将用于更正当前的过滤器状态。
% 滤波器估计中存在一些漂移，可以使用其他传感器（例如GPS）
% 或其他约束（例如道路边界图）进一步校正。
cameraIdx = 1;
for i = 1:numIMUSamples
    % 生成真值轨迹
    [pos(i,:), orient(i,:), vel(i,:), acc(i,:), angvel(i,:)] = traj();

    % Generate accelerometer and gyroscope measurements from the ground truth
    % trajectory values.
    % 从真值数据中生成加速度计和陀螺仪的测量值
    [accelMeas, gyroMeas] = imu(acc(i,:), angvel(i,:), orient(i));

    % Predict the filter state forward one time step based on the
    % accelerometer and gyroscope measurements.
    % 基于测量值，向前预测
    predict(filt, accelMeas, gyroMeas);

    if (1 == mod(i, imuSamplesPerCamera)) && useVO
        % Generate a visual odometry pose estimate from the ground truth
        % values and the visual odometry model.
        % 基于真值和VO模型产生VO姿态估计
        [posVO(cameraIdx,:), orientVO(cameraIdx,:), paramsVO] = ...
            helperVisualOdometryModel(pos(i,:), orient(i,:), paramsVO);

        %
        fusemvo(filt, posVO(cameraIdx,:), RposVO, ...
            orientVO(cameraIdx), RorientVO);

        cameraIdx = cameraIdx + 1;
    end

    [posEst(i,:), orientEst(i,:), velEst(i,:)] = pose(filt);

    % 更新估计的车辆姿态
    helperUpdatePose(estVehicle, posEst(i,:), velEst(i,:), orientEst(i));

    % 更新车辆真值
    helperUpdatePose(groundTruthVehicle, pos(i,:), vel(i,:), orient(i));

    % 更新驾驶场景
    updatePlots(scene); %重新画车辆
    drawnow limitrate;
end

figure
if useVO
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posVO(:,1), posVO(:,2), posVO(:,3), ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'Visual Odometry (VO)', ...
        'Visual-Inertial Odometry (VIO)', 'Location', 'northeast')
else
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', posEst(:,1), posEst(:,2), posEst(:,3), 'LineWidth', 3);
    legend('Ground Truth', 'IMU Pose Estimate')
end
view(-90, 90)
title('Vehicle Position')
xlabel('X (m)')
ylabel('Y (m)')
grid on
