%TODO: ustawić w coppelli wszystkie zasięgi czujniów tak samo - sterować
%bąblem w matlabie

%Obliczanie wektora przyciągającego do celu
function [angle,d_hist,v_des]=HistogramAngle(detStateV, detPointM, v_robot, Ki, delta_t, angle_error)
bubble_boundary=v_robot*delta_t*Ki; %ustal granicę detekcji %to do: zastąpić skalarny bąbel wektorem dla każdego czujnika osobno
angle_sum=0;
d_sum=0;
v_des=0.1;
d=zeros(8,1);
sensor_angles=[90;67.5;45;22.5;-22.5;-45;-67.5;-90];

for i=1:8

   if detStateV(i) == 1  %%czy i-ty czujnik widzi przeszkodę
      dist=CalcDist([0;0],[detPointM(i,1);detPointM(i,3)]);  %oblicz odległość od i-tego czujnika do wykrytej przeszkody
      
      if dist> bubble_boundary
          d(i)=1; %wykryto przeszkodę, ale jest dalej niż bąbel
      else
          d(i)=dist; %wykryto przeszkodę w odległości mniejszej od bąbla
      end
   else
       d(i)=1; %nie wykryto przeszkody
   end
   
    angle_sum=angle_sum+sensor_angles(i)*d(i); %TODO: sprawdzić czy kąty są wagami
    d_sum=d_sum+d(i);
end

%robot jedzie prosto na przeszkodę - dwa środkowe sensory wykryły obecność
if (detStateV(4)==1 && detStateV(5)==1 && detStateV(1)==0 && detStateV(2)==0 && detStateV(3)==0 && detStateV(6)==0 && detStateV(7)==0 && detStateV(8)==0)

angle=2*(angle_sum/d_sum)+sign(randi([-1,1],1))*10; %TODO: sprawdzić mnożnik 
v_des=0.05;                                                  % zastanowić się jak zmieniać kierunek jadąc wprost na ścianę             

else
   angle=2*(angle_sum/d_sum); 
end
d_hist=d;
end