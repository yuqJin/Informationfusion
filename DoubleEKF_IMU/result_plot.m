% t=1;
% figure
% plot(Quat(:,t),'b');
% hold on
% plot(q_EKF2(t,:),'g');
% hold on 
% plot(q_EKF1(t,:),'r');
% hold on
% plot(Quat(:,2));
% hold on
% plot(Quat(:,3));
% hold on 
% plot(Quat(:,4));
roll=[];
pitch=[];
yaw=[];
res=q_EKF2;
rad2dag=180/pi;
for k=1:length(res)
ang_roll=atan2(2*(res(3,k)*res(4,k)+res(1,k)*res(2,k)), (res(1,k)^2-res(2,k)^2-res(3,k)^2+res(4,k)^2));
ang_pitch=asin((2*(res(1,k)*res(3,k)-res(2,k)*res(4,k)))/(res(1,k)^2+res(2,k)^2+res(3,k)^2+res(4,k)^2));
ang_yaw=atan2(2*(res(2,k)*res(3,k)+res(1,k)*res(4,k)),(res(1,k)^2+res(2,k)^2-res(3,k)^2-res(4,k)^2));
if(ang_yaw<0)
    ang_yaw=ang_yaw+360*pi/180;
end
roll=[roll ang_roll];
pitch=[pitch ang_pitch];
yaw=[yaw ang_yaw];
end
roll=roll.*rad2dag;
pitch=pitch.*rad2dag;
yaw=yaw.*rad2dag-270;
createfigure(Euler(:,1),roll,Euler(:,2),pitch,Euler(:,3),yaw);
% figure
% subplot(3,1,1)
% plot(Euler(:,1),'b');
% hold on
% plot(roll(1,:)*rad2dag,'r');
% subplot(3,1,2)
% plot(Euler(:,2),'b');
% hold on
% plot(pitch*rad2dag,'r');
% subplot(3,1,3)
% plot(Euler(:,3),'b');
% hold on
% plot(yaw(1,:)*rad2dag-260,'r');

% res=q_EKF1;
% roll=[];
% pitch=[];
% yaw=[];
% for k=1:length(res)
% ang_roll=atan2(2*(res(3,k)*res(4,k)+res(1,k)*res(2,k)), (res(1,k)^2-res(2,k)^2-res(3,k)^2+res(4,k)^2));
% ang_pitch=asin((2*(res(1,k)*res(3,k)-res(2,k)*res(4,k)))/(res(1,k)^2+res(2,k)^2+res(3,k)^2+res(4,k)^2));
% ang_yaw=atan2(2*(res(2,k)*res(3,k)+res(1,k)*res(4,k)),(res(1,k)^2+res(2,k)^2-res(3,k)^2-res(4,k)^2));
% roll=[roll ang_roll];
% pitch=[pitch ang_pitch];
% yaw=[yaw ang_yaw];
% end
% subplot(3,1,1)
% plot(roll(1,:)*rad2dag,'g');
% subplot(3,1,2)
% plot(yaw(1,:)*rad2dag,'g');
% subplot(3,1,3)
% plot(pitch(1,:)*rad2dag,'g');

%计算MSE
mse_x=0;mse_y=0;mse_z=0;
for i=1:length(roll)-1
    se_x=(Euler(i,1)-roll(i))^2;
    mse_x=mse_x+se_x;
    se_y=(Euler(i,2)-pitch(i))^2;
    mse_y=mse_y+se_y;
    se_z=(Euler(i,3)-yaw(i))^2;
    mse_z=mse_z+se_z;
end
mse_x=mse_x/length(Euler);
mse_x=mse_x^0.5;
mse_y=mse_y/length(Euler);
mse_y=mse_y^0.5;
mse_z=mse_z/length(Euler);
mse_z=mse_z^0.5;




