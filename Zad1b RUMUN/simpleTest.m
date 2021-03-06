

function simpleTest()


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
        sensorID=randperm(5,1); %wylosuj czujnik 
        goal=[5;5]; %to musi być wektor pionowy! 
        perform_search=1;
        path_completed=0;
        angle_error=10;
        d=ones(8,1); %inicjalizacja histogramu
        reorient=0;
        om_des = 0;
%         angle_hist=[0];
%         phi_hist=[0];
%         alpha_hist=[0];
        pause(2);

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
        handle;
          
         while 2 > 1      	%GŁÓWNA PĘTLA SYMULACJI
          
        


                    %pobierz z Copelli położenie robota, kąt i prędkość
                [returnCode, position]=sim.simxGetObjectPosition(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode, orientation]=sim.simxGetObjectOrientation(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode,linearVelocity,rotationVel]=sim.simxGetObjectVelocity(clientID,16,sim.simx_opmode_blocking);         
                r_pose=position+[5, 5, 0]; %przesunięcie układu współrzędnych 
                alfa_abs=orientation(3); %orientacja robota (-pi;pi)
              
                
                %zmniejsz prędkość gdy robot jest już blisko celu
                if CalcDist(r_pose,goal) > 0.3
                v_des=0.1;
                else
                v_des=0.05;
                end
                

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
                

                 
                %jeżeli dystans do celu mniejszy niż 10 cm - cel osiagniety - STOP
                if CalcDist(r_pose,goal) < 0.1 
                    path_completed=1;
                    [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, 0 , sim.simx_opmode_blocking);
                    [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor,0 , sim.simx_opmode_blocking);
                    pause(2)
                    v_des=0;
                    if CalcDist(r_pose,baseLocation') < 0.1
                        fprintf('punkt bazowy osiągnięty! \n');
                        sensorID=randperm(5,1);
%                         goal=[sensorLocation(sensorID,1);sensorLocation(sensorID,2)];
                        goal=[1;9];
                    else
                        fprintf('punkt %d osiągnięty! \n', sensorID);
                       goal=baseLocation';
                    end
                    path_completed=0;
                    reorient=0;
                    
                end
                
                if path_completed==0
                    
                    if reorient == 0
                        [om_des, angle_error,phi]=GoalPursuit2(r_pose(1),r_pose(2),alfa_abs,goal);
                        v_des=0.;
                        if abs(angle_error) < 0.01
                            reorient = 1;
                        end
                    end
                        
                    
                    
           
                     if sum(detStateVector) ==0 && reorient == 1 % jeżeli nie ma żadnej przeszkody - jedź prosto do celu
                        [om_des, angle_error,phi]=GoalPursuit2(r_pose(1),r_pose(2),alfa_abs,goal);
                        v_des=0.1;
                        
%                       if angle_error>1.25 || angle_error<-1.25
%                          v_des=0.01;
%                       end      
                       
                        
                       
                    
                        %wykres pomocniczy
%                         angle_hist=[angle_hist; rad2deg(angle_error)];
%                         phi_hist=[phi_hist;rad2deg(phi)];
%                         alpha_hist=[alpha_hist;rad2deg(alfa)];
                        
                     elseif reorient == 1 %jeżeli jest jakakolowiek przeszkoda na horyzoncie - zajmij się omijaniem przeszkód
%                         print("unikam przeszkod");
                        [angle_correction,d]=HistogramAngle(detStateVector,detPointMatrix,v_des,150,0.1, 1);
                        [~, angle_error,~]=GoalPursuit2(r_pose(1),r_pose(2),alfa_abs,goal);
                        om_des= 0.8*deg2rad(angle_correction);
                        v_des=0.05; %zmniejsz prędkość gdy omiajasz przeszkodę
                        if abs(angle_error)>1.45
                            [om_des, angle_error,~]=GoalPursuit2(r_pose(1),r_pose(2),alfa_abs,goal);
                            v_des=0.;
                        end     
                     
                     end
                
                     
                     
                end
                
                %przytrzymaj robota przez 0.5 sek nieruchomo i obróć w
                %kierunku celu
%                 if reorient <= 5
%                   v_des=0;
%                 end
%                 reorient=reorient+1; %TODO: zerować reorient

                
                figure(1) %rysuj histogram
                 bar(d)
                %wykres pomocniczy
%                 subplot(1,2,1)
%                 plot(angle_hist);
%                 hold on
%                 plot(alpha_hist);
%                 plot(phi_hist);
%                 legend('alg ang','alfa','phi');
%                 hold off
%                 subplot(1,2,2)


               %sterowanie napędami robota na podstawie prędkości liniowej
               %i obrotowej
      
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
