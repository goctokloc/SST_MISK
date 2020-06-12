%Obliczanie wektora przyciągającego do celu
function [Fa_x,Fa_y]=AttractForce(r_pose, goal_pose, k_factor)
Ra=sqrt((r_pose(1)-goal_pose(1))^2+(r_pose(2)-goal_pose(2))^2);
Fa=k_factor*Ra;
r2goal=[goal_pose(1)-r_pose(1);goal_pose(2)-r_pose(2)];
theta=atan2(r2goal(2),r2goal(1));

Fa_x=Fa*cos(theta);
Fa_y=Fa*sin(theta);
end