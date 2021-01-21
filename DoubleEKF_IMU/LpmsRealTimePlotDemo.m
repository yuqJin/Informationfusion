% Lpms Realtime Plot demo
% Summary: 
% This example demonstrate how to obtain
% and plot real time data from LpmsSensor
%
% Author: H.E.Yap
% Date: 2016/07/19    
% Revision: 0.1 
% Copyright: LP-Research Inc. 2016


close
clear all
clc

% Parameters
T = 400;
nCount = 1;
COMPort = 'COM4';
baudrate = 921600;
lpSensor = lpms();
accData = zeros(T,3);

% Connect to sensor
if ( ~lpSensor.connect(COMPort, baudrate) )
    return 
end
disp('sensor connected')

% Set streaming mode
lpSensor.setStreamingMode();

% Loop Plot
figure('doublebuffer','on', ...
       'CurrentCharacter','a', ...
       'WindowStyle','modal')
   
while double(get(gcf,'CurrentCharacter'))~=27
    nData = lpSensor.hasSensorData();
    for i=1:nData
        d = lpSensor.getQueueSensorData();
        if nCount == T
            accData=accData(2:end, :);
        else
            nCount = nCount + 1;
        end
        accData(nCount,:) = d.acc;
    end
    plot(1:T,accData)
    grid on;
    title(sprintf('ts = %fs', d.timestamp))
    drawnow
end

set(gcf,'WindowStyle','normal');
if (lpSensor.disconnect())
    disp('sensor disconnected')
end