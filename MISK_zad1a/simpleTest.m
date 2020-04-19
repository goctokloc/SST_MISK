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
        [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
        
        run('mapa_test_1.m');
        startLocation = [];
        baseLocation=[5 5];
        sensorLocation=[1 9; 9 9; 9.25 2.25; 2.5 3.5; 6.5 2.0];
        sensorID=randperm(5,1); %wylosuj czujnik 
        %dist=CalcDist(startLocation,startLocation);
        dist=0;
        dist2base=0;
        prmSimple = mobileRobotPRM(map,2000);
        figure(1)
        show(map)
        hold on
        prmSimple.ConnectionDistance = 1;
        perform_search=1;
        path_currentpoint=1;
        path_completed=0;
        %show(prmSimple)
        
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
        [returnCode, ultrasonicSensor4]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4', sim.simx_opmode_blocking);
        
        %[returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, 1 , sim.simx_opmode_blocking);
        %[returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor, 1 , sim.simx_opmode_blocking);
        [returnCode, handle]= sim.simxGetObjectHandle(clientID,'Pioneer_p3dx', sim.simx_opmode_blocking);
        [returnCode, handle_0]= sim.simxGetObjectHandle(clientID,'Measuring_station0', sim.simx_opmode_blocking);
        [returnCode, handle_1]= sim.simxGetObjectHandle(clientID,'Measuring_station1', sim.simx_opmode_blocking);
        [returnCode, handle_2]= sim.simxGetObjectHandle(clientID,'Measuring_station2', sim.simx_opmode_blocking);
        [returnCode, handle_3]= sim.simxGetObjectHandle(clientID,'Measuring_station3', sim.simx_opmode_blocking);
        [returnCode, handle_4]= sim.simxGetObjectHandle(clientID,'Measuring_station4', sim.simx_opmode_blocking);
        handle;
                
         %for i=1:50
         while 2 > 1
                
                [returnCode, position]=sim.simxGetObjectPosition(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode, orientation]=sim.simxGetObjectOrientation(clientID,16 , -1, sim.simx_opmode_blocking);
                [returnCode,linearVelocity,rotationVel]=sim.simxGetObjectVelocity(clientID,16,sim.simx_opmode_blocking);         
                r_pose=position+[5, 5, 0];
                alfa_abs=orientation(3);
                
                if (perform_search==1) 
                 dist2base=CalcDist(baseLocation,[r_pose(1) r_pose(2)]);
                 if (dist2base < 0.5)
                     sensorID=randperm(5,1); %wylosuj czujnik
                     startLocation=[double(r_pose(1)), double(r_pose(2))];
                     endLocation=[sensorLocation(sensorID,1),sensorLocation(sensorID,2)];
                     clear path path_pts path_coords
                     path = findpath(prmSimple,startLocation,endLocation);
                     path_done = 1;
                     figure(1)
                     plot(path(:,1),path(:,2),'Color', 'blue','LineStyle','-','Marker','.', 'LineWidth',9)
                     hold on
                     path_currentpoint=1;
                     [path_pts, path_coords]=size(path);
                     fprintf('Planowanie drogi do czujnika:  %d\n', sensorID);
                      if(sensorID == 1)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station2',1,'changeColorGreen',[],[],[],[],sim.simx_opmode_blocking);
                      end
                       if(sensorID == 2)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station3',1,'changeColorGreen',[],[],[],[],sim.simx_opmode_blocking);
                      end
                       if(sensorID == 3)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station0',1,'changeColorGreen',[],[],[],[],sim.simx_opmode_blocking);
                      end
                       if(sensorID == 4)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station4',1,'changeColorGreen',[],[],[],[],sim.simx_opmode_blocking);
                      end
                       if(sensorID == 5)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station1',1,'changeColorGreen',[],[],[],[],sim.simx_opmode_blocking);
                      end
                    
                     
                 else
                     
                     startLocation=[double(r_pose(1)), double(r_pose(2))];
                     clear path path_pts path_coords
                     path = findpath(prmSimple,startLocation,baseLocation);
                     path_currentpoint=1;
                     [path_pts, path_coords]=size(path);
                     fprintf('Planowanie drogi do bazy \n');
                     path_done = 1;
                     sensorID=0;
                 end
                 perform_search=0;
                end
                
                if (path_currentpoint < path_pts-1)
                v_des=0.1;
                end
                if (path_currentpoint == path_pts-1)
                v_des=0.05;
                end
                
                
                if path_completed==1
                    fprintf('punkt %d osiągnięty! \n', sensorID);
                    path_done = 0;
                     if(sensorID == 1)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station2',1,'changeColorRed',[],[],[],[],sim.simx_opmode_blocking);
                     end
                      if(sensorID == 2)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station3',1,'changeColorRed',[],[],[],[],sim.simx_opmode_blocking);
                     end
                      if(sensorID == 3)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station0',1,'changeColorRed',[],[],[],[],sim.simx_opmode_blocking);
                     end
                     if(sensorID == 4)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station4',1,'changeColorRed',[],[],[],[],sim.simx_opmode_blocking);
                     end
                     if(sensorID == 5)
                        [returnCode,outInts, outFloats,outStrings, outBuffer]=sim.simxCallScriptFunction(clientID,'Measuring_station1',1,'changeColorRed',[],[],[],[],sim.simx_opmode_blocking);
                     end
                    
                    [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, 0 , sim.simx_opmode_blocking);
                    [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor,0 , sim.simx_opmode_blocking);
                    pause(2)
                    perform_search=1;
                    path_completed=0;
                end
                
                if path_completed~=1
                [om_des,dist]=PurePursuit(r_pose(1),r_pose(2),alfa_abs,path(path_currentpoint,1),path(path_currentpoint,2));
                end
                
                if dist < 0.1  && path_done == 1 
                    if path_currentpoint < path_pts
                    path_currentpoint=path_currentpoint+1;
                    else
                    path_completed=1;
                    end
                end
                %om_des=0;
                r_wheel=0.0975;
                L=0.331; %sprawdzic dokladny rozstaw kol
                [L_rotvel,R_rotvel]=DiffDrive(v_des,om_des,L,r_wheel);
                [returnCode]= sim.simxSetJointTargetVelocity(clientID, left_Motor, L_rotvel , sim.simx_opmode_blocking);
                [returnCode]= sim.simxSetJointTargetVelocity(clientID, right_Motor, R_rotvel , sim.simx_opmode_blocking);
              
                pause(0.1);
                figure(1)                
                scatter(position(1)+5,position(2)+5, 8,'r', 'filled')
                hold on
            
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
