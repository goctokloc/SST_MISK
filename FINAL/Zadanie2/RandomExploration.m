function [om_des_out] = RandomExploration(d,om_des)
if sum(d) == 8 %jezeli nie ma przeszkod w zasiegu czujnikow
  shuffle=rand(1)-0.5; %wylosuj liczbe z przedzialu -0.5 0.5
  if abs(shuffle)>0.4 %jezeli modul z liczby losowej jest wiekszy niz 0.4, dodaj czynnik losowy do predkosci obrotowej
        om_des_out=om_des+shuffle*0.66;
  else
      om_des_out=om_des;
  end
end
end

