close all
figure(1)
hold on
axis equal
map = importdata('/home/npereira/Udacity_Self_Driving_Cars/Term2/Particle_Filter/data/map_data.txt');
plot(map(:,1),map(:,2),'b*')
gt = importdata('/home/npereira/Udacity_Self_Driving_Cars/Term2/Particle_Filter/data/gt_data.txt')


observation = importdata('/home/npereira/Udacity_Self_Driving_Cars/Term2/Particle_Filter/data/observation/observations_000340.txt');
robot.pose = [gt(340,:)]
robot.model = 'n223'
robot.radius = 50
robot.colour = 'r'

%% transform observations from local to global
for i=1:size(observation,1)
    observation(i,:) = LocalToGlobal (observation(i,1), observation(i,2), robot.pose(1), robot.pose(2), robot.pose(3))
end
    
%% test gt data
%  gt   x | y | veredict
% 340   + | - | OK
% 894   + | + | OK
% 1528  - | - | OK
% 1612  - | + | OK

plot (observation(:,1),observation(:,2),'og')

plot_handle = []
cleanFlag = 0
printFlag = 1
DrawRobot(gca,plot_handle,robot,cleanFlag,printFlag)