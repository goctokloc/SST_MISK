% Make sure to have the server side running in CoppeliaSim: 
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!



function simpleTest()
    r_pose=[0 0 0];
    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
            
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
%         [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
%         if (res==sim.simx_return_ok)
%             fprintf('Number of objects in the scene: %d\n',length(objs));
%         else
%             fprintf('Remote API function call returned with error code: %d\n',res);
%         end
        
        
        startLocation = [];
        baseLocation=[5 5];
        sensorLocation=[1 9; 9 9; 9.25 2.25; 2.5 3.5; 6.5 2.0];
        sensorID=randperm(5,1); %wylosuj czujnik 
        %dist=CalcDist(startLocation,startLocation);
        dist=0;
        dist2base=0;

        perform_search=1;
        path_currentpoint=1;
        path_completed=0;
        %show(prmSimple)
        angle_error=0;
        
        pause(2);
    
        % Now retrieve streaming data (i.e. in a non-blocking fashion):
        %t=clock;
        %startTime=t(6);
        %currentTime=t(6);
        %sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming); % Initialize streaming
        %while (currentTime-startTime < 5)   
        %    [returnCode,data]=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer); % Try to retrieve the streamed data
        %    if (returnCode==sim.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        %        fprintf('Mouse position x: %d\n',data); % Mouse position x is actualized when the cursor is over CoppeliaSim's window
        %    end
        %    t=clock;
        %    currentTime=t(6);
        %end
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
                
         %for i=1:50
         while 2 > 1      	
                

                %if detectionState == 1
%              detectedPointLeft;
%              XLobstacle=(detectedPointLeft(3)*cosd(10)+0.2)*double(detectionStateLeft);
%              YLobstacle=(detectedPointLeft(1)*cosd(10)+0.02)*double(detectionStateLeft);
%              XRobstacle=(detectedPointRight(3)*cosd(10)+0.2)*double(detectionStateRight);
%              YRobstacle=(detectedPointRight(1)*cosd(10)-0.02)*double(detectionStateRight);
%              Xobstacle=double(1/(detectionStateLeft+detectionStateRight))*(double(XLobstacle+XRobstacle))
%              Yobstacle=double(1/(detectionStateLeft+detectionStateRight))*(double(YLobstacle+YRobstacle))
%              distance=sqrt(double(Xobstacle^2+Yobstacle^2));
%              rad2deg(atan2(double(Yobstacle),double(Xobstacle)));
             %end
             
                [returnCode, position]=sim.simxGetObjectPosition(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode, orientation]=sim.simxGetObjectOrientation(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode,linearVelocity,rotationVel]=sim.simxGetObjectVelocity(clientID,16,sim.simx_opmode_blocking);         
                r_pose=position+[5, 5, 0];
                alfa_abs=orientation(3);
                
                goal=[1;1];
                if CalcDist(r_pose,goal) > 0.3
                v_des=0.1;
                else
                v_des=0.05;
                end
                

                
                
                [returnCode,detectionState_1, detectedPoint_1,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor1,sim.simx_opmode_blocking);
                [returnCode,detectionState_2, detectedPoint_2,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor2,sim.simx_opmode_blocking);
                [returnCode,detectionState_3, detectedPoint_3,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor3,sim.simx_opmode_blocking);
                [returnCode,detectionState_4, detectedPoint_4,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor4,sim.simx_opmode_blocking);
                [returnCode,detectionState_5, detectedPoint_5,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor5,sim.simx_opmode_blocking);
                [returnCode,detectionState_6, detectedPoint_6,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor6,sim.simx_opmode_blocking);
                [returnCode,detectionState_7, detectedPoint_7,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor7,sim.simx_opmode_blocking);
                [returnCode,detectionState_8, detectedPoint_8,detectedObjectHandle,detectedSurfaceNormalVector]=sim.simxReadProximitySensor(clientID,ultrasonicSensor8,sim.simx_opmode_blocking);
                
                detStateVector=[detectionState_1;detectionState_2;detectionState_3;detectionState_4;detectionState_5;detectionState_6; detectionState_7;detectionState_8];
                detPointMatrix=[detectedPoint_1;detectedPoint_2;detectedPoint_3;detectedPoint_4;detectedPoint_5;detectedPoint_6;detectedPoint_7;detectedPoint_8];
                

                 
                
                if CalcDist(r_pose,goal) < 0.1
                    path_completed=1;
                    fprintf('punkt %d osiągnięty! \n', sensorID);
                    [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, 0 , sim.simx_opmode_blocking);
                    [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor,0 , sim.simx_opmode_blocking);
                    pause(2)
                    v_des=0;
                end
                
                if path_completed==0
                %[om_des,w]=PurePursuit2(r_pose(1),r_pose(2),alfa_abs,goal,angle_correction);
                %[rad2deg(alfa_abs), rad2deg(phi)]
%                      if sum(detStateVector) ==0
%                         [om_des, angle_error]=GoalPursuit(r_pose(1),r_pose(2),alfa_abs,goal);
%                         d=ones(8,1);
%                      else
                        [angle_correction,d]=HistogramAngle(detStateVector,detPointMatrix,v_des,150,0.1, 0);
                        angle_correction
                        om_des= 0.8*deg2rad(angle_correction);
                        %v_des=0.05*min(d)+0.05;
                     end
                
%                  end
             
                        figure(1)
                        bar(d)
                
                %om_des=0;
                r_wheel=0.0975;
                L=0.331; %sprawdzic dokladny rozstaw kol
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
