function createfigure(Y1, Y2, Y3, Y4, Y5, Y6)
%CREATEFIGURE(Y1, Y2, Y3, Y4, Y5, Y6)
%  Y1:  y 数据的向量
%  Y2:  y 数据的向量
%  Y3:  y 数据的向量
%  Y4:  y 数据的向量
%  Y5:  y 数据的向量
%  Y6:  y 数据的向量

%  由 MATLAB 于 14-Jan-2021 22:51:40 自动生成

% 创建 figure
figure1 = figure;

% 创建 subplot
subplot1 = subplot(3,1,1,'Parent',figure1);
hold(subplot1,'on');

% 创建 plot
plot(Y1,'DisplayName','Real roll','Parent',subplot1,'LineWidth',1.5,...
    'Color',[0 0 1]);

% 创建 plot
plot(Y2,'DisplayName','Estimated roll','Parent',subplot1,'LineWidth',1.5,...
    'LineStyle','-.',...
    'Color',[1 0 0]);

% 创建 ylabel
ylabel('Roll(deg)','HorizontalAlignment','center');

% 创建 xlabel
xlabel('Time(ms)','HorizontalAlignment','center');

box(subplot1,'on');
hold(subplot1,'off');
% 设置其余坐标区属性
set(subplot1,'FontName','Times New Roman','FontSize',12,'XTick',...
    [0 200 400 600 800 1000 1200],'XTickLabel',...
    {'0','200','400','600','800','1000','1200'});
% 创建 legend
legend(subplot1,'show');

% 创建 subplot
subplot2 = subplot(3,1,2,'Parent',figure1);
hold(subplot2,'on');

% 创建 plot
plot(Y3,'DisplayName','Real pitch','Parent',subplot2,'LineWidth',1.5,...
    'Color',[0 0 1]);

% 创建 plot
plot(Y4,'DisplayName','Estimated pitch','Parent',subplot2,'LineWidth',1.5,...
    'LineStyle','-.',...
    'Color',[1 0 0]);

% 创建 ylabel
ylabel('Pitch(deg)','HorizontalAlignment','center');

% 创建 xlabel
xlabel({'Time(ms)',''},'HorizontalAlignment','center');

box(subplot2,'on');
hold(subplot2,'off');
% 设置其余坐标区属性
set(subplot2,'FontName','Times New Roman','FontSize',12,'XTickLabel',...
    {'0','200','400','600','800','1000','1200'},'YTickLabel',{'0','100','200'});
% 创建 legend
legend1 = legend(subplot2,'show');
set(legend1,...
    'Position',[0.752015501580497 0.489102870544674 0.146744188131288 0.0540586641345473]);

% 创建 subplot
subplot3 = subplot(3,1,3,'Parent',figure1);
hold(subplot3,'on');

% 创建 plot
plot(Y5,'DisplayName','Real yaw','Parent',subplot3,'LineWidth',1.5,...
    'Color',[0 0 1]);

% 创建 plot
plot(Y6,'DisplayName','Estimated yaw','Parent',subplot3,'LineWidth',1.5,...
    'LineStyle','-.',...
    'Color',[1 0 0]);

% 创建 ylabel
ylabel('Yaw(deg)','HorizontalAlignment','center');

% 创建 xlabel
xlabel('XLabel','HorizontalAlignment','center');

box(subplot3,'on');
hold(subplot3,'off');
% 设置其余坐标区属性
set(subplot3,'FontName','Times New Roman','FontSize',12,'XTickLabel',...
    {'0','200','400','600','800','1000','1200'},'YTickLabel',{'0','100','200'});
% 创建 legend
legend(subplot3,'show');

