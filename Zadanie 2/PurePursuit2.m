%ALGORYTM PODĄŻĄNIA ZA TRAJEKTORIĄ
%zwraca żądaną prędkość obrotową proporcjonalną do błędu orientacji oraz
%dystans robota do bieżącego punktu 
function [om_des,w] = PurePursuit2(x_abs,y_abs,alfa_abs, goal, angle_correction)
r=[x_abs;y_abs]; %położenie absolutne robota
p=goal; %położenie celu
w=p-r; 
phi=atan2(w(2),w(1));
angle_error=phi-alfa_abs;

% Rot=[cos(alfa_abs), -sin(alfa_abs); sin(alfa_abs) cos(alfa_abs)]; %macierz rotacji z ukł. abs do ukł. robota];
% w=inv(Rot)*(p-r); %współrzędne punktu trajektorii w układzie robota
% dis=norm(w);    %odległość robota od punktu
% phi=rad2deg(atan2(w(2),w(1)));
total_angle=angle_error+deg2rad(angle_correction);
% om_des=1.0*total_angle;
om_des= 0.8*deg2rad(angle_correction);
%om_des=0.8*total_angle;

end