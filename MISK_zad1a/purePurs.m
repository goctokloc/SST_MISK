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
                     
                     path_currentpoint=1;
                     [path_pts, path_coords]=size(path);
                
                  
                     
                 else
                     
                     startLocation=[double(r_pose(1)), double(r_pose(2))];
                     clear path path_pts path_coords
                     path = findpath(prmSimple,startLocation,baseLocation);
                     path_currentpoint=1;
                     [path_pts, path_coords]=size(path);
                   
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
