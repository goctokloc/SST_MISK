%ALGORYTM PODĄŻĄNIA ZA TRAJEKTORIĄ
%zwraca żądaną prędkość obrotową proporcjonalną do błędu orientacji oraz
%dystans robota do bieżącego punktu 
function [om_des, angle_error,phi,alfa] = GoalPursuit(x_abs,y_abs,alfa_abs, goal)
r=[x_abs;y_abs]; %położenie absolutne robota
p=goal; %położenie celu
w=p-r; 
phi=atan2(w(2),w(1));
alfa=0;
% if alfa_abs < 0
%     
%     alfa=alfa_abs+3.1415;
% end


angle_error=phi-alfa_abs;
%angle_error=phi-alfa;
% if angle_error > 3.1415/2 
%     angle_error=3.1415/2;
% end
%     
% if angle_error < -3.1415/2 
%     angle_error= -3.1415/2;
% end

% Rot=[cos(alfa_abs), -sin(alfa_abs); sin(alfa_abs) cos(alfa_abs)]; %macierz rotacji z ukł. abs do ukł. robota];
% w=inv(Rot)*(p-r); %współrzędne punktu trajektorii w układzie robota
% dis=norm(w);    %odległość robota od punktu
% phi=rad2deg(atan2(w(2),w(1)));

om_des= 0.4*angle_error;
%om_des=0.8*total_angle;

end