%zmniejszaj poziom baterii z każdą iteracją, gdy osiągnie 0, wystaw status
%"wymaga interwencji"

%wejścia: licznik iteracji (informacja o czasie), aktualny zapas każdej z
%baterii
%wyjścia: wektor statusów czujników (1=wymaga interwencji, 0=nie wymaga)
%zaktualizowany stan baterii
function [sensorStatus, sensorBattery_new] = CheckSensorBattery(sensorBattery_current)
sensorStatus=zeros(5,1);
sensorBattery_new=sensorBattery_current;
for n=1:size(sensorBattery_current,2)
    if sensorBattery_current(n) > 0 %jeśli bateria nie jest rozładowana, rozładowuj dalej
    sensorBattery_new(n)=sensorBattery_current(n)-1;
    end
    if sensorBattery_current(n) == 0 %jesli baterię rozładowano, poinformuj o potrzebie interwencji
        sensorStatus(n)=1;
    end
end

end