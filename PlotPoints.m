%% Fast ploting of any data in the same figure without having to clean all plots
% @ axesHandle (axes to plot to, e.g. 'gca' get current axes)
% @ plot_handle (handle to store the plot)
% @ points_matrix (handle to store the plot)
% @ color
% @ cleanFlag
% @ printFlag

function plot_handle = PlotPoints(axesHandle,plot_handle,points_matrix,colour,cleanFlag,printFlag)
    if nargin < 4
        disp('ERROR. Insuficient number of argumets in PlotPoints');
    end
	if ~exist('cleanFlag','var')
        cleanFlag = 1;
    end
    if ~exist('printFlag','var')
        printFlag = 1;
    end
    if ishandle(plot_handle) & cleanFlag
        delete(plot_handle);
    end
    if printFlag ==1
        if size(points_matrix,1)>0  && size(points_matrix,2)>1 
            plot_handle = plot(axesHandle,points_matrix(:,1),points_matrix(:,2),colour);
        end
    end
end