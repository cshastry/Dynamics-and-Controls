function M = plot_motion( x,t,trajectory_handle,parameters )
%PLOT_MOTION Summary of this function goes here
%   Detailed explanation goes here
scale = 1.0;
x = scale*x.';
[Desired_axes,~] = arrayfun(trajectory_handle,t,'UniformOutput',0);

fig = figure('units','normalized','outerposition',[0 0 1 1]);
ax = gca;
ax.XLimMode = 'manual';
ax.YLimMode = 'manual';
ax.ZLimMode = 'manual';

ax.XLim = [-1.5 1.5];
ax.YLim = [-1.5 1.5];
ax.ZLim = [-1.5 1.5];

ax.DataAspectRatioMode = 'manual';
ax.DataAspectRatio = [1 1 1];

ax.PlotBoxAspectRatioMode = 'manual';
ax.PlotBoxAspectRatio = [1 1 1]

ax.View =[45 45];
ax.CameraViewAngle = 10;
ax.CameraViewAngleMode = 'manual';
grid on
hold on

for i = 1:size(parameters.v,2)
    Cone([0 0 0].',parameters.v(:,i),[0,tan(parameters.theta(i))],100,'r',0,0);
    alpha(0.3)
    line([0 parameters.v(1,i)],[0 parameters.v(2,i)],[0 parameters.v(3,i)].','Color','yellow','LineStyle','-');
end

% drawnow limitrate
    

for i=1:length(x)
    
    plot3(x(1,i),x(2,i),x(3,i),'Color','blue','Marker','.','MarkerSize',5);
    %r = line([0 y(4,i)],[0 y(5,i)],[0 y(6,i)],'Color','green','LineStyle',':');
    
    % Current R
    xaxis = line([0 x(1,i)],[0 x(2,i)],[0 x(3,i)],'Color','blue','LineStyle','-','MarkerSize',25,'LineWidth',2); 
    yaxis = line([0 x(4,i)],[0 x(5,i)],[0 x(6,i)],'Color','blue','LineStyle','-','MarkerSize',25,'LineWidth',2);
    zaxis = line([0 x(7,i)],[0 x(8,i)],[0 x(9,i)],'Color','blue','LineStyle','-','MarkerSize',25,'LineWidth',2);
    
    x_label = text(x(1,i),x(2,i),x(3,i),'X','FontSize',14,'Color','black');
    y_label = text(x(4,i),x(5,i),x(6,i),'Y','FontSize',14,'Color','black');
    z_label = text(x(7,i),x(8,i),x(9,i),'Z','FontSize',14,'Color','black');
    
    % Desired R
    xaxis_des = line([0 Desired_axes{i}(1,1)],[0 Desired_axes{i}(2,1)],[0 Desired_axes{i}(3,1)],'Color','green','LineStyle','-','MarkerSize',25,'LineWidth',2); 
    yaxis_des = line([0 Desired_axes{i}(1,2)],[0 Desired_axes{i}(2,2)],[0 Desired_axes{i}(3,2)],'Color','green','LineStyle','-','MarkerSize',25,'LineWidth',2);
    zaxis_des = line([0 Desired_axes{i}(1,3)],[0 Desired_axes{i}(2,3)],[0 Desired_axes{i}(3,3)],'Color','green','LineStyle','-','MarkerSize',25,'LineWidth',2);
    
    xdes_label = text(Desired_axes{i}(1,1),Desired_axes{i}(2,1),Desired_axes{i}(3,1),'X_{des}','FontSize',14,'Color','black');
    ydes_label = text(Desired_axes{i}(1,2),Desired_axes{i}(2,2),Desired_axes{i}(3,2),'Y_{des}','FontSize',14,'Color','black');
    zdes_label = text(Desired_axes{i}(1,3),Desired_axes{i}(2,3),Desired_axes{i}(3,3),'Z_{des}','FontSize',14,'Color','black');
    
    pause(eps)
    
    delete(xaxis);
    delete(yaxis);
    delete(zaxis);
    
    delete(x_label);
    delete(y_label);
    delete(z_label);
    
    delete(xaxis_des);
    delete(yaxis_des);
    delete(zaxis_des);
    
    delete(xdes_label);
    delete(ydes_label);
    delete(zdes_label);
    
    %M(i) = getframe;
end


end

