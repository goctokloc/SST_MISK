%Obliczanie wektora przyciągającego do celu
function [angle,d_hist]=HistogramAngle(detStateV, detPointM, v_robot, Ki, delta_t, angle_error)
bubble_boundary=v_robot*delta_t*Ki; %ustal granicę detekcji
angle_sum=0;
d_sum=0;
d=zeros(8,1);
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
   
    angle_sum=angle_sum+sensor_angles(i)*d(i);
    d_sum=d_sum+d(i);
end


if (detStateV(4)==1 && detStateV(5)==1 && detStateV(1)==0 && detStateV(2)==0 && detStateV(3)==0 && detStateV(6)==0 && detStateV(7)==0 && detStateV(8)==0)

angle=2*(angle_sum/d_sum)+sign(angle_error)*15;

else
   angle=2*(angle_sum/d_sum); 
end
d_hist=d;
end