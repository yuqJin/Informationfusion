%默认的IMU模型：理想的加速度计和理想的陀螺仪。
IMU=imuSensor;
%仿真输入
trueAcceleration = [1 0 0];
trueAngularVelocity = [1 0 0];
[accelerometerReadings, gyroscopeReadings] = IMU(trueAcceleration, trueAngularVelocity);

GPS=gpsSensor;
truePosition = [1 0 0];
trueVelocity = [1 0 0];
[LLA,velocity,groundspeed,course]=GPS(truePosition,trueVelocity);