function createfigure(polyshape1, polyshape2, polyshape3, polyshape4, XMatrix1, YMatrix1, polyshape5, polyshape6, polyshape7, polyshape8, polyshape9, polyshape10, polyshape11, polyshape12, polyshape13, polyshape14, polyshape15, polyshape16, polyshape17, polyshape18, X1, Y1, Size1, XMatrix2, YMatrix2, X2, Y2, X3, Y3, Y4)
%CREATEFIGURE(polyshape1, polyshape2, polyshape3, polyshape4, XMatrix1, YMatrix1, polyshape5, polyshape6, polyshape7, polyshape8, polyshape9, polyshape10, polyshape11, polyshape12, polyshape13, polyshape14, polyshape15, polyshape16, polyshape17, polyshape18, X1, Y1, Size1, XMatrix2, YMatrix2, X2, Y2, X3, Y3, Y4)
%  POLYSHAPE1:  polyshape object
%  POLYSHAPE2:  polyshape object
%  POLYSHAPE3:  polyshape object
%  POLYSHAPE4:  polyshape object
%  XMATRIX1:  matrix of plot x data
%  YMATRIX1:  matrix of plot y data
%  POLYSHAPE5:  polyshape object
%  POLYSHAPE6:  polyshape object
%  POLYSHAPE7:  polyshape object
%  POLYSHAPE8:  polyshape object
%  POLYSHAPE9:  polyshape object
%  POLYSHAPE10:  polyshape object
%  POLYSHAPE11:  polyshape object
%  POLYSHAPE12:  polyshape object
%  POLYSHAPE13:  polyshape object
%  POLYSHAPE14:  polyshape object
%  POLYSHAPE15:  polyshape object
%  POLYSHAPE16:  polyshape object
%  POLYSHAPE17:  polyshape object
%  POLYSHAPE18:  polyshape object
%  X1:  vector of scatter x data
%  Y1:  vector of scatter y data
%  SIZE1:  vector of scatter size data
%  XMATRIX2:  matrix of plot x data
%  YMATRIX2:  matrix of plot y data
%  X2:  vector of plot x data
%  Y2:  vector of plot y data
%  X3:  vector of plot x data
%  Y3:  vector of plot y data
%  Y4:  vector of plot y data

%  Auto-generated by MATLAB on 17-Dec-2022 14:13:52

% Create figure
figure;

% Create axes
axes1 = axes('Position',...
    [0.130729169410373 0.676855895196506 0.119791663922961 0.251091697305769]);
hold(axes1,'on');

% Create plot
plot(polyshape1,'FaceColor',[0 0 0],'FaceAlpha',0.35);

% Create plot
plot(polyshape2,'FaceColor',[0 0 0],'FaceAlpha',0.35);

% Create plot
plot(polyshape3,'FaceColor',[1 0 0],'FaceAlpha',0.1);

% Create plot
plot(polyshape4,'FaceColor',[1 0 0],'FaceAlpha',0.1);

% Create multiple line objects using matrix input to plot
plot1 = plot(XMatrix1,YMatrix1,'MarkerSize',10,'Marker','o',...
    'LineStyle','none');
set(plot1(1),'MarkerFaceColor',[0 0.447058823529412 0.741176470588235],...
    'Color',[0 0.447058823529412 0.741176470588235]);
set(plot1(2),...
    'MarkerFaceColor',[0.392156862745098 0.831372549019608 0.0745098039215686],...
    'Color',[0.392156862745098 0.831372549019608 0.0745098039215686]);

% Create plot
plot(polyshape5,'FaceColor',[1 0 0],'FaceAlpha',0.117649);

% Create plot
plot(polyshape6,'FaceColor',[1 0 0],'FaceAlpha',0.117649);

% Create plot
plot(polyshape7,'FaceColor',[1 0 0],'FaceAlpha',0.16807);

% Create plot
plot(polyshape8,'FaceColor',[1 0 0],'FaceAlpha',0.16807);

% Create plot
plot(polyshape9,'FaceColor',[1 0 0],'FaceAlpha',0.2401);

% Create plot
plot(polyshape10,'FaceColor',[1 0 0],'FaceAlpha',0.2401);

% Create plot
plot(polyshape11,'FaceColor',[1 0 0],'FaceAlpha',0.343);

% Create plot
plot(polyshape12,'FaceColor',[1 0 0],'FaceAlpha',0.343);

% Create plot
plot(polyshape13,'FaceColor',[1 0 0],'FaceAlpha',0.49);

% Create plot
plot(polyshape14,'FaceColor',[1 0 0],'FaceAlpha',0.49);

% Create plot
plot(polyshape15,'FaceColor',[1 0 0],'FaceAlpha',0.7);

% Create plot
plot(polyshape16,'FaceColor',[1 0 0],'FaceAlpha',0.7);

% Create plot
plot(polyshape17,'FaceColor',[1 0 0]);

% Create plot
plot(polyshape18,'FaceColor',[1 0 0]);

% Create ylabel
ylabel('$x_2$ (m)','HorizontalAlignment','center','FontWeight','bold',...
    'FontSize',20,...
    'Interpreter','latex');

% Create xlabel
xlabel('$x_1$ (m)','HorizontalAlignment','center','FontSize',20,...
    'Interpreter','latex');

% Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[-2.1 2.1]);
% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[-2.1 2.1]);
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);
% Create axes
axes2 = axes('Position',...
    [0.3171875 0.67993006993007 0.175578457446809 0.24506993006993]);
hold(axes2,'on');

% Create scatter
scatter(X1,Y1,Size1,'DisplayName','obstacles',...
    'MarkerEdgeColor',[0.7 0.7 0.7],...
    'MarkerFaceColor',[0.7 0.7 0.7],...
    'Marker','square');

% Create multiple line objects using matrix input to plot
plot2 = plot(XMatrix2,YMatrix2,'MarkerSize',10,'Marker','o',...
    'LineStyle','none');
set(plot2(1),'DisplayName','start',...
    'MarkerFaceColor',[0 0.447058823529412 0.741176470588235],...
    'Color',[0 0.447058823529412 0.741176470588235]);
set(plot2(2),'DisplayName','goal',...
    'MarkerFaceColor',[0.466666666666667 0.674509803921569 0.188235294117647],...
    'Color',[0.466666666666667 0.674509803921569 0.188235294117647]);

% Create plot
plot(X2,Y2,'DisplayName','trajectory','LineWidth',2,...
    'Color',[0 0.447058823529412 0.741176470588235]);

% Create ylabel
ylabel('$q_2$ (rad)','HorizontalAlignment','center','FontSize',20,...
    'Interpreter','latex');

% Create xlabel
xlabel('$q_1$(rad)','HorizontalAlignment','center','FontWeight','bold',...
    'FontSize',20,...
    'FontName','Adobe Devanagari',...
    'Interpreter','latex');

% Uncomment the following line to preserve the X-limits of the axes
% xlim(axes2,[-0.1 6.28318530717959]);
% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes2,[-0.1 6.28318530717959]);
box(axes2,'on');
hold(axes2,'off');
% Set the remaining axes properties
set(axes2,'FontSize',15,'LineWidth',1.5,'TickLabelInterpreter','latex',...
    'XTick',[0 2.09439510239319 4.18879020478639 6.28318530717959],'XTickLabel',...
    {'0','$\pi/3$','$2\pi/3$','$2\pi$'},'YTick',...
    [0 2.09439510239319 4.18879020478639 6.28],'YTickLabel',...
    {'0','$\pi/3$','$2\pi/3$','$2\pi$'});
% Create legend
legend1 = legend(axes2,'show');
set(legend1,...
    'Position',[0.189337262986603 0.949169620519002 0.232089372699279 0.0305676848086728],...
    'Orientation','horizontal',...
    'Interpreter','latex',...
    'FontSize',15);

% Create axes
axes3 = axes('Position',...
    [0.13 0.497043512043513 0.362765957446809 0.102587412587412]);
hold(axes3,'on');

% Create plot
plot(X3,X2,'LineWidth',2);

% Create ylabel
ylabel('$q_1$ (rad)','HorizontalAlignment','center','FontSize',20,...
    'Interpreter','latex');

% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes3,[-0.5 4]);
box(axes3,'on');
hold(axes3,'off');
% Set the remaining axes properties
set(axes3,'FontSize',15,'LineWidth',1.5,'XTickLabel',...
    {'','','','','','','','','','',''});
% Create axes
axes4 = axes('Position',...
    [0.13 0.36241733575067 0.362765957446809 0.102587412587412]);
hold(axes4,'on');

% Create plot
plot(X3,Y2,'LineWidth',2);

% Create ylabel
ylabel('$q_2$ (rad)','HorizontalAlignment','center','FontSize',20,...
    'Interpreter','latex');

% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes4,[-0.5 4]);
box(axes4,'on');
hold(axes4,'off');
% Set the remaining axes properties
set(axes4,'FontSize',15,'LineWidth',1.5,'XTickLabel',...
    {'','','','','','','','','','',''});
% Create axes
axes5 = axes('Position',...
    [0.13 0.224424156090824 0.362765957446809 0.102587412587412]);
hold(axes5,'on');

% Create plot
plot(X3,Y3,'LineWidth',2);

% Create ylabel
ylabel('$\dot{q_1}$ $(\frac{rad}{s})$','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',20,...
    'Interpreter','latex');

% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes5,[-0.5 6]);
box(axes5,'on');
hold(axes5,'off');
% Set the remaining axes properties
set(axes5,'FontSize',15,'LineWidth',1.5,'XTickLabel',...
    {'','','','','','','','','','',''});
% Create axes
axes6 = axes('Position',...
    [0.13 0.0909203142536482 0.362765957446809 0.102587412587413]);
hold(axes6,'on');

% Create plot
plot(X3,Y4,'LineWidth',2);

% Create ylabel
ylabel('$\dot{q_2}$ $(\frac{rad}{s})$','HorizontalAlignment','center',...
    'FontSize',20,...
    'Interpreter','latex');

% Create xlabel
xlabel('time (s)','HorizontalAlignment','center','FontSize',20,...
    'Interpreter','latex');

% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes6,[-5.5 5.5]);
box(axes6,'on');
hold(axes6,'off');
% Set the remaining axes properties
set(axes6,'FontSize',15,'LineWidth',1.5);
