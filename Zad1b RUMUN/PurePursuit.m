%ALGORYTM PODĄŻĄNIA ZA TRAJEKTORIĄ
%zwraca żądaną prędkość obrotową proporcjonalną do błędu orientacji oraz
%dystans robota do bieżącego punktu 
function [om_des, dis] = PurePursuit(x_abs,y_abs,alfa_abs, path_x, path_y)
r=[x_abs;y_abs]; %położenie absolutne robota
p=[path_x;path_y]; %położenie punktu trajektorii
Rot=[cos(alfa_abs), -sin(alfa_abs); sin(alfa_abs) cos(alfa_abs)]; %macierz rotacji z ukł. abs do ukł. robota];
w=inv(Rot)*(p-r); %współrzędne punktu trajektorii w układzie robota
dis=norm(w);    %odległość robota od punktu
phi=atan2(w(2),w(1));   
om_des=0.8*phi;

end