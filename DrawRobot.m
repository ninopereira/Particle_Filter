%% Draw the robot
% robot: structure with radius, model, pose and color
% e.g.(robot.model = 'n223; robot.radius=0.15
% robot.pose=[2 2 0.1]% array [x, y theta])
function plot_handle = DrawRobot(axesHandle,plot_handle,robot,cleanFlag,printFlag)
    if ~exist('cleanFlag','var')
       cleanFlag = 1;
    end
    if ~exist('printFlag','var')
       printFlag = 1;
    end
    
    if strcmp(robot.model,'n223')||strcmp(robot.model,'n276')
        plot_handle = Draw223(axesHandle,plot_handle,robot,cleanFlag,printFlag);
    else
        plot_handle = Draw277(axesHandle,plot_handle,robot,cleanFlag,printFlag);
    end
end