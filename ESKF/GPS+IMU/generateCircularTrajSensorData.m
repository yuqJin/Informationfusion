function generateCircularTrajSensorData()
% Generate the circular trajectory sensor data used in the "Pose Estimation
% From Asynchronous Sensors" demo. 

%   Copyright 2018 The MathWorks, Inc.    

Fs = 100;   % Maximum MARG rate
gpsFs = 5;  % Maximum GPS rate.
ratio = Fs./gpsFs;
refloc = [42.2825 -72.3430 53.0352];
magField = [19.5281 -5.0741 48.0067];

Fs = 100;
trajData = circleTraj(Fs);

trajOrient = trajData.Orientation;
trajVel = trajData.Velocity;
trajPos = trajData.Position;
trajAcc = trajData.Acceleration;
trajAngVel = trajData.AngularVelocity;

%% Setup the random number generator
rng(1)

%% IMU Simulation 
imu = imuSensor('accel-gyro-mag', 'SampleRate', Fs);
imu.MagneticField = magField;

% Accelerometer
imu.Accelerometer.MeasurementRange =  19.6133;
imu.Accelerometer.Resolution = 0.0023928;
imu.Accelerometer.NoiseDensity = 0.0012356;

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.0625);
imu.Gyroscope.ConstantBias = deg2rad([3.125 3.125 7]);
imu.Gyroscope.AxesMisalignment = [1.5 0 3];
imu.Gyroscope.NoiseDensity = deg2rad(0.025);

% Magnetometer
imu.Magnetometer.MeasurementRange = 1000;
imu.Magnetometer.Resolution = 0.1;
imu.Magnetometer.NoiseDensity = 0.1/ sqrt(50);

[accel, gyro, mag] = imu(trajAcc, trajAngVel, trajOrient);

%% GPS Simulation
gps = gpsSensor('UpdateRate', gpsFs);
gps.ReferenceLocation = refloc;     
gps.DecayFactor = 0.5;              
gps.HorizontalPositionAccuracy = 1.6;   
gps.VerticalPositionAccuracy =  1.6;
gps.VelocityAccuracy = 0.1;           

[llaGPSRate, gpsvelGPSRate] = gps( trajPos(1:ratio:end,:), ...
    trajVel(1:ratio:end, :)); %#ok<BDSCI>

lla = upsampleGPS(llaGPSRate, ratio);
gpsvel = upsampleGPS(gpsvelGPSRate, ratio);

save('CircularTrajectorySensorData.mat', 'Fs', 'gpsFs', 'refloc', ...
    'magField', 'trajData', 'accel', 'gyro', 'mag', 'lla', 'gpsvel');

end

function trajData = circleTraj(fs)
% Generate a circular motion trajectory

N = 10000; 
r = 10; % (m)
speed = 2.5; % (m/s)
center = [0, 0]; % (m)
initialYaw = 90; % (degrees)
numRevs = 20;

% Define angles theta and corresponding times of arrival t.
revTime = 2*pi*r / speed;
theta = (0:pi/2:2*pi*numRevs).';
t = linspace(0, revTime*numRevs, numel(theta)).';

% Define position.
x = r .* cos(theta) + center(1);
y = r .* sin(theta) + center(2);
z = zeros(size(x));
position = [x, y, z];

% Define orientation.
yaw = theta + deg2rad(initialYaw);
yaw = mod(yaw, 2*pi);
pitch = zeros(size(yaw));
roll = zeros(size(yaw));
orientation = quaternion([yaw, pitch, roll], 'euler', ...
    'ZYX', 'frame');

% Generate trajectory.
groundTruth = waypointTrajectory('SampleRate', fs, ...
    'Waypoints', position, ...
    'TimeOfArrival', t, ...
    'Orientation', orientation);


pos = zeros(N,3);
q = quaternion.zeros(N,1);
vel = zeros(N,3);
acc = zeros(N,3);
av = zeros(N,3);


for ii=1:N
    [pos(ii,:), qtmp, vel(ii,:), acc(ii,:), av(ii,:)] = groundTruth();
    q(ii) = qtmp;
end


trajData.Orientation = q;
trajData.Velocity = vel;
trajData.Position = pos;
trajData.Acceleration = acc;
trajData.AngularVelocity = av;
end

function y = upsampleGPS(x, ratio)
%UPSAMPLEGPS Sample and hold GPS input to higher rate
xup = repmat(x, 1,1,ratio);
xp = permute(xup,[3 1 2]);
y = reshape(xp, [], 3);
end