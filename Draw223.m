function plot_handle = Draw223(axesHandle,plot_handle,robot,cleanFlag,printFlag);
    if ~exist('cleanFlag','var')
       cleanFlag = 1;
    end
    if ~exist('printFlag','var')
       printFlag = 1;
    end
    
    x = robot.pose(1);
    y = robot.pose(2);
    theta = robot.pose(3);

    robotWidth = 2*robot.radius;
    robotHeight = 2*robot.radius;

    x1 = x - robotWidth/2;
    y1 = y - robotHeight/2;

    %% draw a circle
    ang=0:pi/8:2*pi; 
    x_circle=robot.radius*cos(ang);
    y_circle=robot.radius*sin(ang);
    
    line_pointing_forward = robotWidth/5; %thin line pointing the forward direction
    x_line = [robotWidth/2 0 0];
    y_line = [0 line_pointing_forward/2 -line_pointing_forward/2 ];
    xy = [x_line x_circle;y_line y_circle];
    
    % rotate at (0,0)
    rotationArray = [cos(theta), -sin(theta);...
                     sin(theta), cos(theta)];
    xy=rotationArray*xy;

    % translate to desired center point
    xy(1,:) = xy(1,:) + x;
    xy(2,:) = xy(2,:) + y;

    robot_xy = [xy(1,:) ;...
                xy(2,:) ]';
    
	%% plot 223 robot shape
    plot_handle = PlotPoints(axesHandle,plot_handle,robot_xy,robot.colour,cleanFlag,printFlag);
end
