function [] = aroundWall2(clientID, vrep, States, dis, left_Motor, right_Motor)
    if (States(1) ==1 &&  States(2) ~= 1 && States(5) ~= 1)%sum(States) == 1 && 
        if(norm(dis(1))<0.5)
            [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,-0.5, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.5,vrep.simx_opmode_oneshot);       
        end
    elseif (States(1) == 1 && States(2) == 1 && States(5) == 1 )
            [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,-0.5, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.5,vrep.simx_opmode_oneshot);
            %pause(0.7)     
    elseif (States(2) == 1 && States(5) == 1 && States(1) ~= 1)
        if(dis(2)>0.5 && dis(5)>0.5)
            [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,-0.5, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.5,vrep.simx_opmode_oneshot);
        elseif(dis(2)<0.5&& dis(5)<0.5 && (dis(2)- dis(5)<-0.002))
            [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,-0.5, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1.5,vrep.simx_opmode_oneshot);
         elseif(dis(2)<0.5&& dis(5)<0.5 && (dis(2)- dis(5)>0.002) ) 
            [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1.5, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,-0.5,vrep.simx_opmode_oneshot);
        else
            [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1.0, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1.0,vrep.simx_opmode_oneshot);
        end
        
    else
        [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,1.0, vrep.simx_opmode_oneshot);
        [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,1.0,vrep.simx_opmode_oneshot);
    end

end