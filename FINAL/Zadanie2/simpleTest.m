
function simpleTest()

    close all;
    disp('Program started');
    
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
         
        %inicjalizacja zmiennych
        r_pose=[0 0 0]; 
        baseLocation=[5 5];
        sensorLocation=[1 9; 9 9; 9.25 2.25; 2.5 3.5; 6.5 2.0];
        sensorBattery_default=randperm(200,5)+30; %wylosuj fabryczny czas działania na baterii [0.1s]
        sensorBattery_current=sensorBattery_default; %początkowo baterie mają pełną pojemność
        sensorStatus=zeros(5,1); %wektor stanów baterii -> 0=nie wymaga akcji, 1=akcja wymagana
        sensorID=randperm(5,1); %wylosuj czujnik
        measuring_StationID=[2;3;0;4;1]; %numeracja stacji czujnikowych w Coppelii
        sensorRange=2.0; %zasięg sygnału czujników
        goal=[0; 0]; %to musi być wektor pionowy! 
        path_completed=0;
        angle_error=0;
        visit_counter=0; %licznik odwiedzonych sensorów, jeśli więcej niż 3, jedź do bazy
        d=ones(8,1); %inicjalizacja histogramu
        random_wander=1; %flaga sygnalizująca chaotyczne poruszanie sie
        station_name='Measuring_station';
        during_reorient=0; %flaga sygnalizujaca proces obracania robota w miejscu
        visitRequest=0;
        
        odczyty = {'czujnik nr ', '-'; 'PM 10: ',' brak danych '; 'PM 2.5: ',' brak danych '; 'NO2: ', 'brak danych'; 'O3: ', 'brak danych'};

        pause(2);
            
        %handle do elementów z Coppelii
        [returnCode, left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
        [returnCode, right_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor1]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor2]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor3]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor4]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor5]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor6]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor7]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7', sim.simx_opmode_blocking);
        [returnCode, ultrasonicSensor8]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8', sim.simx_opmode_blocking);
        [returnCode, handle]= sim.simxGetObjectHandle(clientID,'Pioneer_p3dx', sim.simx_opmode_blocking);
        
        [returnCode, handle]= sim.simxGetObjectHandle(clientID,'Pioneer_p3dx', sim.simx_opmode_blocking);
        [returnCode, handle_0]= sim.simxGetObjectHandle(clientID,'Measuring_station0', sim.simx_opmode_blocking);
        [returnCode, handle_1]= sim.simxGetObjectHandle(clientID,'Measuring_station1', sim.simx_opmode_blocking);
        [returnCode, handle_2]= sim.simxGetObjectHandle(clientID,'Measuring_station2', sim.simx_opmode_blocking);
        [returnCode, handle_3]= sim.simxGetObjectHandle(clientID,'Measuring_station3', sim.simx_opmode_blocking);
        [returnCode, handle_4]= sim.simxGetObjectHandle(clientID,'Measuring_station4', sim.simx_opmode_blocking);
      
        f= figure('Name','Panel informacyjny','NumberTitle','off','Resize','off'); %uruchom dashboard
        f.Position=[100 100 300 400];
        
         while 2 > 1      	%GŁÓWNA PĘTLA SYMULACJI
                

                %pobierz z Copelli położenie robota, kąt i prędkość
                [returnCode, position]=sim.simxGetObjectPosition(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode, orientation]=sim.simxGetObjectOrientation(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode,linearVelocity,rotationVel]=sim.simxGetObjectVelocity(clientID,16,sim.simx_opmode_blocking);         
                r_pose=position+[5, 5, 0]; %przesunięcie układu współrzędnych 
                alfa_abs=orientation(3); %orientacja robota (-pi;pi)
                true_velocity=norm(linearVelocity);
                
                % elementy okna graficznego
                data = {'sensor 1',sensorBattery_current(1),sensorStatus(1);'sensor 2',sensorBattery_current(2),sensorStatus(2);'sensor 3',sensorBattery_current(3), sensorStatus(3);...
                    'sensor 4',sensorBattery_current(4),sensorStatus(4);'sensor 5',sensorBattery_current(5),sensorStatus(5)};
                uit = uitable(f);
                uit.Data=data;
                uit.ColumnName = {'czujnik','bateria','zgłoszenie'};
                uit.Position = [20 20 (f.Position(3)-40) 115];
                c = uicontrol(f,'Style','text','FontSize',15);
                c.Position = [20 f.Position(4)-80 f.Position(3)-40 80];
                if random_wander == 1
                    zadanie='eksploracja mapy';
                elseif random_wander==0 && visit_counter==3
                    zadanie='powrót do bazy';
                elseif random_wander==0 && visit_counter<3 && visitRequest==1
                    pom_str={'w drodze do czujnika ',num2str(sensorID)};
                    zadanie=strjoin(pom_str,'');
                end
                
                 c.String = {'aktualne zadanie: ', zadanie };
                
                 text= uicontrol(f,'Style','text','FontSize',15);
                 text.Position = [20 c.Position(2)-40 f.Position(3)-40 40];
                 pom_str={'odwiedzone czujniki: ',num2str(visit_counter), ' z 3'};
                 text.String = {strjoin(pom_str,'')};
                 
                odczyty_tab = uitable(f,'Data',odczyty);
%                 odczyty_tab.Data=odczyty;
                odczyty_tab.ColumnName = {'parametr','wartość'};
                odczyty_tab.Position = [20 uit.Position(2)+uit.Position(4) (f.Position(3)-40) 115];
                 
                 
                 
                %aktualizuj stan baterii oraz sprawdzaj czy wamagana jest interwencja
                [sensorStatus, sensorBattery_current] = CheckSensorBattery(sensorBattery_current);
                
                
                %kolorowanie czujników na zielono
                
                for q=1:5
                   if sensorStatus(q) == 1
                        C={'Measuring_station',num2str(measuring_StationID(q))};
                        station_name=strjoin(C,'');
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,station_name,1,'changeColorGreen',[],[],[],[],sim.simx_opmode_blocking);
                   end
                    
                end
                
              
%                 sensorBattery_current
%                 sensorStatus
                

              
                if visit_counter < 3 %jeżeli robot odwiedził mniej niż 3 czujniki, niech szuka dalej
                    
                    %sprawdzaj czy robot jest w zasięgu jakiegokolwiek czujnika wymagającego interwencji, jeśli tak to podaj ID
                    
                
                     %jeżeli robot jest w pobliżu czujnika do interwecji, ustaw go za cel
                     if visitRequest==1 
                         goal=[sensorLocation(sensorID,1);sensorLocation(sensorID,2)];
                         random_wander=0; %przestań poruszać się losowo
                     else
                         random_wander=1; %zacznij poruszać się losowo
                         %TO DO: zastanowić się jak definiować cel 
                         [visitRequest, sensorID]=CheckSensorProximity(r_pose,sensorLocation,sensorStatus,sensorRange);
                     end
                 
                else %jeżeli przepełniony bufor robota - niech wraca do bazy: ignoruj prośby o wizytę, bez losowego przemieszczania
                    
                    visitRequest=0;
                    goal=[5;5];
                    random_wander=0;
                    sensorID=0;
                                   
                end
                

%                 visitRequest
%                 sensorID
%                 goal
%                 visit_counter

                %pobierz dane z Copelli dot. czujników zbliżeniowych
                
                [returnCode,detectionState_1, detectedPoint_1,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor1,sim.simx_opmode_blocking);
                [returnCode,detectionState_2, detectedPoint_2,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor2,sim.simx_opmode_blocking);
                [returnCode,detectionState_3, detectedPoint_3,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor3,sim.simx_opmode_blocking);
                [returnCode,detectionState_4, detectedPoint_4,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor4,sim.simx_opmode_blocking);
                [returnCode,detectionState_5, detectedPoint_5,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor5,sim.simx_opmode_blocking);
                [returnCode,detectionState_6, detectedPoint_6,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor6,sim.simx_opmode_blocking);
                [returnCode,detectionState_7, detectedPoint_7,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor7,sim.simx_opmode_blocking);
                [returnCode,detectionState_8, detectedPoint_8,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor8,sim.simx_opmode_blocking);
                %wektor zawierający stan wykrycia przeszkód na
                %poszczególnych sensorach
                detStateVector=[detectionState_1;detectionState_2;detectionState_3;detectionState_4;detectionState_5;detectionState_6; detectionState_7;detectionState_8];
                %współrzędne wykrytych punktów 
                detPointMatrix=[detectedPoint_1;detectedPoint_2;detectedPoint_3;detectedPoint_4;detectedPoint_5;detectedPoint_6;detectedPoint_7;detectedPoint_8];
                

                 if random_wander==0 %jeżeli robot ma zdefiniowany konkretny cel
                     
                        %zmniejsz prędkość gdy robot jest już blisko celu
                    if CalcDist(r_pose,goal) > 0.5
                        v_des=0.1;
                        avoidance_enabled=1;
                    else
                        v_des=0.05;
                        avoidance_enabled=0;
                    end
                    
                    %jeżeli dystans do celu mniejszy niż 20 cm - cel osiagniety - STOP
                    if CalcDist(r_pose,goal) < 0.2 
                        path_completed=1;
                        fprintf('punkt %d osiągnięty! \n', sensorID);
                        [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, 0 , sim.simx_opmode_blocking);
                        [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor,0 , sim.simx_opmode_blocking);
                        pause(2)
                        v_des=0;
                        
                        if goal(1) ==5 && goal(2)==5 %jeżeli robot dotarł do celu i jest nim baza - zresetuj licznik odwiedzonych czujników
                            visit_counter=0; 
                        else
                            odczyty(1,2)={num2str(sensorID)}; odczyty(2,2)={num2str(rand(1)*200)}; odczyty(3,2)={num2str(rand(1)*100)}; odczyty(4,2)={num2str(rand(1)*100)}; odczyty(5,2)={num2str(rand(1)*100)};
                            visit_counter=visit_counter+1; %jeżeli robot dotarł do kolejnego czujnika, zwiększ licznik o 1
                            sensorBattery_current(sensorID)=1000;  %po dojechaniu do celu naładuj baterię do 1000
                            C={'Measuring_station',num2str(measuring_StationID(sensorID))};
                            station_name=strjoin(C,'');
                            [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,station_name,1,'changeColorRed',[],[],[],[],sim.simx_opmode_blocking); 
                            during_reorient=1; %po odebraniu danych z czujnika zasygnalizuj potrzebę obrotu
                        end           
                        

                        visitRequest=0; %zdejmij żądanie jazdy do celu
                        path_completed=0;
                        

                        
                        
                    end
                
                    if path_completed==0

                         if sum(detStateVector) ==0 % jeżeli nie ma żadnej przeszkody - jedź prosto do celu
                            [om_des, angle_error,phi]=GoalPursuit2(r_pose(1),r_pose(2),alfa_abs,goal);

                            %wykres pomocniczy
    %                         angle_hist=[angle_hist; rad2deg(angle_error)];
    %                         phi_hist=[phi_hist;rad2deg(phi)];
    %                         alpha_hist=[alpha_hist;rad2deg(alfa)];
    
                         else %jeżeli jest jakakolowiek przeszkoda na horyzoncie - zajmij się omijaniem przeszkód
                             if avoidance_enabled==1
                                [angle_correction,d,v_des]=HistogramAngle(detStateVector,detPointMatrix,v_des,150,0.1, 0);
                                om_des= 0.8*deg2rad(angle_correction);
                             end
                         end
                    end
                 end
                 
                 if random_wander == 1 %jeżeli robot porusza się losowo - wylosuj kierunek , ewentualnie omijaj przeszkody
                        v_des=0.1;
                     [angle_correction,d,v_des]=HistogramAngle(detStateVector,detPointMatrix,v_des,150,0.1, 1);
                      om_des= 0.8*deg2rad(angle_correction);
                      
                      %om_des=RandomExploration(d,om_des);
                      if sum(d) == 8 
                          shuffle=rand(1)-0.5;
                          if abs(shuffle)>0.4
                                om_des=om_des+shuffle*0.66;
                          end
                      end
                 end



                  %obróć robota w kierunku stacji bazowej po odebraniu
                  %danych z czujnika
               if during_reorient == 1
                [om_des, angle_error,phi]=GoalPursuit2(r_pose(1),r_pose(2),alfa_abs,[5;5]);
                 v_des=0;
                 om_des=om_des/2;
                        if abs(angle_error) < 0.15
                            during_reorient = 0;
                            v_des=0.1;
                        end 
               end
               
               %wycofaj po zderzeniu z przeszkodą
               if (true_velocity < 0.02 && sum(d)<8 &&during_reorient==0)
                   v_des=-0.1;
                   om_des=0.5;
               end
      
               
               %sterowanie napędami robota na podstawie prędkości liniowej i obrotowej
               
                r_wheel=0.0975; %promień koła
                L=0.331; %rozstaw kół
                [L_rotvel,R_rotvel]=DiffDrive(v_des,om_des,L,r_wheel);
                [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, L_rotvel , sim.simx_opmode_blocking);
                [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor, R_rotvel , sim.simx_opmode_blocking);
              
            pause(0.1);
        end
        [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, 0, sim.simx_opmode_blocking);
        [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor, 0, sim.simx_opmode_blocking);
        
        
        % Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);

        % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end
