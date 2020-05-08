%Obliczanie wektora przyciągającego do celu
function [Fr_x,Fr_y]=RepulseForce(Xobstacle, Yobstacle, alfa_abs, threshold, z_factor)
distance=sqrt(Xobstacle^2+Yobstacle^2);
if distance > threshold
  Fr=0;
else
Fr=z_factor*((1/distance) - (1/distance)^3)*(1/distance)^3;
end

r2goal=[goal_pose(1)-r_pose(1);goal_pose(2)-r_pose(2)];
theta=atan2(r2goal(2),r2goal(1));

Fr_x=Fa*cos(theta);
Fr_y=Fa*sin(theta);
end