%oblicz prędkość obrotową kół robota na podstawie:
%v_des - żądana predkosc liniowa (ukł. robota)
%om_des - żądana predkosc obrotowa (ukł. robota)
%L - odległość między kołami !!w metrach!!
%r_wheel - promień koła robota !!w metrach!!
function [omega_lw, omega_rw]= DiffDrive(v_des, om_des, L, r_wheel) 
v_r=v_des+(L/2)*om_des; %prędkość liniowa koło prawe
v_l=v_des-(L/2)*om_des; %prędkość liniowa koło lewe

omega_rw=v_r/r_wheel;   %oblicznowa prędkość obrotowa koła prawego
omega_lw=v_l/r_wheel;   %oblicznowa prędkość obrotowa koła lewego
                                                              
end