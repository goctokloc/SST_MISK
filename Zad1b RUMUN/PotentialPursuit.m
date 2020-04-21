
function [om_des] = PotentialPursuit(alfa_abs,Fx,Fy)
phi_des=atan2(Fy,Fx);
angle_error=phi_des-alfa_abs;
om_des=0.8*angle_error;
end