run('mapa_test_1.m')
% mapa = binaryOccupancyMap(6,6,10);
% x0=[0];
% y0=[0];
% x1=[5.8];
% y1=[5.8];
% x3=2.9;
% y3=3.1;
% 
% setOccupancy(mapa,[x0 y0], ones(60,2))
% setOccupancy(mapa,[x0 y0], ones(2,60))
% setOccupancy(mapa,[x1 y0], ones(60,2))
% setOccupancy(mapa,[x0 y1], ones(2,60))
% setOccupancy(mapa,[x3 y0], ones(29,2))
% setOccupancy(mapa,[x3 y3], ones(29,2))
% 
start = [3.35, 4.72, 0];
goal = [1, 9, pi];
show(map);

%% A* HYBRID
primitive_length=1;
minrad=(2*primitive_length)/pi;
validator = validatorOccupancyMap;
validator.Map = map;
planner = plannerHybridAStar(validator,'MinTurningRadius',minrad,'MotionPrimitiveLength',primitive_length,'NumMotionPrimitives',3);
refpath = plan(planner,start,goal);
Astar_length=pathLength(refpath)
show(planner)
%% RRT
hold on
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')
r = 0.2;
plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-' )
plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-' )
hold off

bounds = [map.XWorldLimits; map.YWorldLimits; [0 20]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.1;

stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.1;

planner = plannerRRT(ss, stateValidator);
planner.MaxConnectionDistance = 0.2;
planner.MaxIterations = 3000000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
rng(0,'twister')

[pthObj, solnInfo] = plan(planner, start, goal);
RRT_length=pathLength(pthObj)
figure
show(map)
hold on

% Search tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

% Interpolate and plot path
%interpolate(pthObj,300)
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)

% Show the start and goal in the grid map
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')
hold off
%% PRM

prmSimple = mobileRobotPRM(map,2000);
prmSimple.ConnectionDistance = 1;
startLocation = [3.35, 4.72];
endLocation = [1 9];
path = findpath(prmSimple,startLocation,endLocation);
PRM_lenght=0;
[punkty wspolrzedne]=size(path);
for i=1:(punkty-1)
    PRM_lenght=PRM_lenght+sqrt((path(i+1,1)-path(i,1))^2 + (path(i+1,2)-path(i,2))^2);
end
PRM_lenght
figure
show(prmSimple)
%figure
%show(map)