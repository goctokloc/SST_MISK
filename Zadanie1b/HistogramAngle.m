%Obliczanie wektora przyciągającego do celu
function [angle]=HistogramAngle(detStateV, detPointM, v_robot, Ki, delta_t)
bubble_boundary=v_robot*delta_t*Ki; %ustal granicę detekcji
angle_sum=0;
d_sum=0;
sensor_angles=[90;67.5;45;22.5;-22.5;-45;-67.5;-90];
%sensor_angles=[90;50;30;10;-10;-30;-50;-90];
for i=1:8

   if detStateV(i) == 1
      dist=CalcDist([0;0],[detPointM(i,1);detPointM(i,3)]);
      
      if dist> bubble_boundary
          d(i)=1;
      else
       %d(i)=dist/bubble_boundary;
       d(i)=dist;
      end
   else
       d(i)=1;
   end
   
%    if (i==1 || i==8) 
%        d(i)=d(i)/2;
%    end
%    
%       if (i==2 || i==7) 
%        d(i)=d(i)/2;
%    end
   
    angle_sum=angle_sum+sensor_angles(i)*d(i);
    d_sum=d_sum+d(i);
end

angle=4*(angle_sum/d_sum);

figure(1)
histogram(d,8)


end