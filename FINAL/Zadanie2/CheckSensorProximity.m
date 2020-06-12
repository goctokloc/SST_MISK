%sprawdzaj cały czas, czy robot znajdue się dostatecznie blisko aktywnego
%czujnia

%wejścia: bieżące położenie robota, położenia czujników, stan czujników,
%zasięg czujników
%wyjścia: visitRequest = 1- jest w pobliżu aktywnego czujnika, 0 - jest za daleko
%sensorID - numer czujnika który spełnia kryterium odległości
function [visitRequest, sensorID] = CheckSensorProximity(r_pose,sensorLocation,sensorStatus, range)

dist2sensor=0; %zmienna pomocnicza
visitRequest=0;
sensorID=0;
for n=1:size(sensorStatus,1)
    
    if sensorStatus(n) == 1 %jeżeli czujnik wysyła prośbę o interwencję
        
     
        dist2sensor=CalcDist(r_pose(1:2),sensorLocation(n,:));
        
        if dist2sensor <=range %jeżeli robot jest w zasięgu czujnika, wystaw flagę i podaj ID odpowiedniego czujnika
            visitRequest=1;
            sensorID=n;
        else
            %visitRequest=0;   %jeśli robot jest poza zasięgiem czujnia, schowaj flagę zajętości 
            %sensorID=0;
        end
        
    end
end

end