% Lpms Data recording demo
% Summary: 
% This example demonstrate how to obtain
% and plot real time data from LpmsSensor
%
% Author: H.E.Yap
% Date: 2016/07/19    
% Revision: 0.1 
% Copyright: LP-Research Inc. 2016

close
clear
clear all
clc


% Parameters
nData = 1000;   % number of data to record
nCount = 1;
COMPort = 'COM4';
baudrate = 115200;
lpSensor = lpms();
ts = zeros(nData,1);
accData = zeros(nData,3);

% Connect to sensor
if ( ~lpSensor.connect(COMPort, baudrate) )
    return 
end
disp('sensor connected')

% Set streaming mode
lpSensor.setStreamingMode();

disp('Accumulating sensor data')
while nCount <= nData
    d = lpSensor.getQueueSensorData();
    if (~isempty(d))
        ts(nCount) = d.timestamp;
        accData(nCount,:) = d.acc;
        nCount=nCount + 1;
    end
end
disp('Done')
if (lpSensor.disconnect())
    disp('sensor disconnected')
end

plot(ts-ts(1), accData);
xlabel('timestamp(s)');
ylabel('Acc(g)');
grid on

