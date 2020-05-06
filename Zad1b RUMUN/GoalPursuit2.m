%ALGORYTM PODĄŻĄNIA ZA TRAJEKTORIĄ
%zwraca żądaną prędkość obrotową proporcjonalną do błędu orientacji oraz
%dystans robota do bieżącego punktu 
function [om_des, angle_error,phi] = GoalPursuit2(x_abs,y_abs,alfa_abs, goal)
r=[x_abs;y_abs]; %położenie absolutne robota
p=goal; %położenie celu
w=p-r; 
phi=atan2(w(2),w(1));

if (phi >= 0 && alfa_abs>=0)
angle_error=phi-alfa_abs;
end

if (phi >= 0 && alfa_abs<0)
angle_error=phi+alfa_abs;
end

if (phi < 0 && alfa_abs>=0)
angle_error=phi+alfa_abs;
end

if (phi < 0 && alfa_abs<0)
angle_error=phi-alfa_abs;
end

om_des= 0.5*angle_error;

end
