data=csvread('data_2.csv',1,0);
data=data;
%Gyro_w=data(:,7:9)*0.017453;
Gyro_w=data(:,7:9)*pi/180;
Quat=data(:,16:19);
Acc=-data(:,4:6);
Mag=data(:,10:12);
Euler=data(:,13:15);