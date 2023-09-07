landmark_x1 = [];
landmark_y1 = [];
landmark_x2 = [];
landmark_y2 = [];


%  simExtRemoteApiStart(19999)
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    [r, robotPose1] = vrep.simxGetObjectHandle(clientID,'Robotpose1',vrep.simx_opmode_blocking);
    [r, robotPose2] = vrep.simxGetObjectHandle(clientID,'Pose2',vrep.simx_opmode_blocking);
    [~,left_Motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [~,front_sensor] = vrep.simxGetObjectHandle(clientID,'front_prox',vrep.simx_opmode_blocking);
    [~,front_right] = vrep.simxGetObjectHandle(clientID,'front_right',vrep.simx_opmode_blocking);
    [~,front_left] = vrep.simxGetObjectHandle(clientID,'front_left',vrep.simx_opmode_blocking);
    [~,rear_left]  = vrep.simxGetObjectHandle(clientID,'rear_left',vrep.simx_opmode_blocking);
    [~,rear_right] = vrep.simxGetObjectHandle(clientID,'rear_right',vrep.simx_opmode_blocking);
    %%ROBOT2
    [~,left_Motor2] = vrep.simxGetObjectHandle(clientID,'motor_left_2',vrep.simx_opmode_blocking);
    [~,right_Motor2] = vrep.simxGetObjectHandle(clientID,'motor_right_2',vrep.simx_opmode_blocking);
    [~,front_sensor2] = vrep.simxGetObjectHandle(clientID,'front_prox_2',vrep.simx_opmode_blocking);
    [~,front_right2] = vrep.simxGetObjectHandle(clientID,'front_right_2',vrep.simx_opmode_blocking);
    [~,front_left2] = vrep.simxGetObjectHandle(clientID,'front_left_2',vrep.simx_opmode_blocking);
    [~,rear_left2]  = vrep.simxGetObjectHandle(clientID,'rear_left_2',vrep.simx_opmode_blocking);
    [~,rear_right2] = vrep.simxGetObjectHandle(clientID,'rear_right_2',vrep.simx_opmode_blocking);

    %% First call sensing
    
    sensorHle = [front_sensor,front_right,front_left,rear_left,rear_right];
    sensorHle2 = [front_sensor2,front_right2,front_left2,rear_left2,rear_right2];
    [~,~] = vrep.simxGetFloatSignal(clientID, 'landmark_x1', vrep.simx_opmode_streaming );
    [~,~] = vrep.simxGetFloatSignal(clientID, 'landmark_y1', vrep.simx_opmode_streaming );
    [~,~] = vrep.simxGetFloatSignal(clientID, 'landmark_x2', vrep.simx_opmode_streaming );
    [~,~] = vrep.simxGetFloatSignal(clientID, 'landmark_y2', vrep.simx_opmode_streaming );
    for i = 1:length(sensorHle)
        vrep.simxReadProximitySensor(clientID,sensorHle(i),vrep.simx_opmode_streaming);
        vrep.simxReadProximitySensor(clientID,sensorHle2(i),vrep.simx_opmode_streaming);
    end
    [~, dis_R] =vrep.simxCheckDistance(clientID,robotPose1, robotPose2, vrep.simx_opmode_streaming );
    
%% 
while(true)
    [~, dis_R] =vrep.simxCheckDistance(clientID,robotPose1, robotPose2, vrep.simx_opmode_buffer );
    [States, dis] = readSen(clientID,vrep,sensorHle);
    [States2, dis2] = readSen(clientID, vrep, sensorHle2);
    % aroundwall behavior
     aroundWall2(clientID, vrep, States, dis, left_Motor, right_Motor);
     aroundWall1(clientID, vrep, States2, dis2, left_Motor2, right_Motor2);
     
    [r1,x1] =  vrep.simxGetFloatSignal(clientID, 'landmark_x1', vrep.simx_opmode_buffer);
    [~,y1] =  vrep.simxGetFloatSignal(clientID, 'landmark_y1', vrep.simx_opmode_buffer);
    [r2,x2] =  vrep.simxGetFloatSignal(clientID, 'landmark_x2', vrep.simx_opmode_buffer);
    [~,y2] =  vrep.simxGetFloatSignal(clientID, 'landmark_y2', vrep.simx_opmode_buffer);
    %% có tín hiệu về khoảng cách  sẽ tạo 1 landmark
    % tạo landmark xe 1 
    if (r1 == 0)
        landmark_x1 = [landmark_x1, x1];
        landmark_y1 = [landmark_y1, y1];
        vrep.simxClearFloatSignal(clientID, 'landmark_x1', vrep.simx_opmode_oneshot);
        vrep.simxClearFloatSignal(clientID, 'landmark_y1', vrep.simx_opmode_oneshot);
        pause(0.05)
    end
    % tạo landmark xe 2
    if r2 == 0
        landmark_x2 = [landmark_x2, x2];
        landmark_y2 = [landmark_y2, y2];
        vrep.simxClearFloatSignal(clientID, 'landmark_x2', vrep.simx_opmode_oneshot);
        vrep.simxClearFloatSignal(clientID, 'landmark_y2', vrep.simx_opmode_oneshot);
        pause(0.05)
    end
    %% Khoảng cách quá gần sẽ dừng 2 robot
     if dis_R < 1 && dis_R > 0
         [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor,0, vrep.simx_opmode_oneshot);
         [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_oneshot);
         [~] = vrep.simxSetJointTargetVelocity(clientID,left_Motor2,0, vrep.simx_opmode_oneshot);
         [~] = vrep.simxSetJointTargetVelocity(clientID,right_Motor2,0,vrep.simx_opmode_oneshot);
        break
     end

end
end

%%  Save
    save landmark landmark_x1 landmark_y1 landmark_x2 landmark_y2
%% func
function [States, dis] = readSen(clientID, vrep, sensorHle)
States = zeros(1,length(sensorHle));
dis = zeros(1,length(sensorHle));
for i = 1:length(sensorHle)
    [~, state,point,~,~] = vrep.simxReadProximitySensor(clientID,sensorHle(i),vrep.simx_opmode_buffer);
    if state == 1
        States(i) = state;
        dis(i) = norm(point);
    end 
end

end


