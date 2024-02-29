%   main_program.m
%   Main program for the control of manipulators
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2022
%   2022/02/04%--------------------------------------------------------------------------
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%--------------------------------------------------------------------------
clear
% need to choose the arm to control
robot_name = 'MTB_2DOF';
%robot_name = 'MTB_3DOF';

% need to choose the gripper/hand
hand_name = 'BarrettHand';

% Creation of a communication class object with the simulator
% Try to connect to simulator
% Output:
% sim - pointer to class simulator_interface
% error_sim = 1 - impossible to connect to simulator
[sim,error_sim] = simulator_interface();
if(error_sim==1)
    return;
end

%Creation of a communication class object with the manipulator arm
% Input:
% sim - pointer to class simulator_interface
% robot_name - name of arm CoppeliaSim model
% hand_name - name of hand CoppeliaSim model
% Output:
% robot_arm - pointer to class arm_interface
% error_man = 1 - impossible to connect to simulator
[robot_arm,error_man] = arm_interface(sim,robot_name,hand_name); 
if error_man == 1
    sim.terminate();
    return;
end

[error,timestep] = sim.get_simulation_timestep(); 
%get time step value (normally dt=50ms)
if error == 1
    sim.terminate();
    return;
end
[error,nJoints,Links,DistanceHand,MinPositionJoint,MaxPositionJoint] = robot_arm.get_RobotCharacteristics();
%nJoints - number of arm joints.
%Links - dimensions of the links between the axes of rotation
%DistanceHand - distance between the tip of the manipulator and the palm of 
% the hand
%MinPositionJoint - array with minimum position for joint 1-6
%MaxPositionJoint - array with maximum position for joint 1-6
if error == 1
    sim.terminate();
    return;
end

%-------------------------------------------------------------------------
error = robot_arm.open_hand(); %Initialize with hand opened
if error == 1
    sim.terminate();
    return;
end

start = tic;
m=1;
stop=0;

% -------User variables
goToTarget = 0;
graspObj = 0;
releaseObj = 0;
goToStart = 1;
goToCheckpoint = 0;
error_grip = 0;
error_gripStatus = 0;
exit = 0;
wait_count = 0;
while stop==0
    %----------------------------------------------------------------------
    %% Robot interface
    % set and get information to/from vrep
    % avoid do processing in between ensure_all_data and trigger_simulation
    sim.ensure_all_data();
    
    % ReadArmJoints - get joint value (rad) for arm
    [error,ReadArmJoints] = robot_arm.get_joints();
    if error == 1
        sim.terminate();
        return;
    end
    
    % armPosition - get robot position
    [error,armPosition] = robot_arm.get_robot_position();
    if error == 1,
        sim.terminate();
        return;
    end
    
    % targetPosition - get target position
    [error,targetPosition]=sim.get_target_position();
    if error == 1,
        sim.terminate();
        return;
    end

    % obstaclePosition - get target position
    [error,obstaclePosition]=sim.get_obstacle_position();
    if error == 1
        sim.terminate();
        return;
    end
    
    %get simulation time
    [error,sim_time] = sim.get_simulation_time();
    if error == 1
        sim.terminate();
        return;
    end
    
    %trigger simulation step
    sim.trigger_simulation();
    %----------------------------------------------------------------------
    % --- YOUR CODE --- %
    if goToTarget == 1
        disp('GO TO TARGET');
        %Inverse Kinematics - Send values for joints
        pe = [targetPosition(1) targetPosition(2)]';  
        S=-1; % Elbow right
        % S=1; % Elbow left
        [error_inv_kin, qout]=InvKin_planar_2DOF_geo(pe,Links, S, MinPositionJoint, MaxPositionJoint);
        qout_deg = qout*180/pi;
        armJoints(1) = qout(1);
        armJoints(2) = qout(2);
        error = robot_arm.set_joints(armJoints); %send value for arm Joints in rad
        
        qinput=[ReadArmJoints(1)  ReadArmJoints(2)]';
        [pe]=DirKin_planar_2DOF(qinput,Links);
        distX = abs(pe(1) - targetPosition(1)); 
        distY = abs(pe(2) - targetPosition(2));
        if distX <= 0.001 && distY <= 0.001
            goToTarget = 0;
            graspObj = 1;
        end
        
    elseif goToCheckpoint == 1
        disp('GO TO CHECKPOINT');
        theta1_rad = -60*pi/180; % rad
        theta2_rad = 90*pi/180; % rad
        % qinput=[theta1_rad  theta2_rad]';

        % [pe]=DirKin_planar_2DOF(qinput,Links);
        
        armJoints(1)=theta1_rad;
        armJoints(2)=theta2_rad;
        error = robot_arm.set_joints(armJoints);

        distX = (ReadArmJoints(1) - theta1_rad); 
        distY = (ReadArmJoints(2) - theta2_rad);
        if distX <= 0.1 && distY <= 0.2
            goToCheckpoint = 0;
            releaseObj = 1;
        end

    elseif goToStart == 1
        disp('GO TO START');
        armJoints(1) = 0;
        armJoints(2) = 0;
        error = robot_arm.set_joints(armJoints);
        distX = abs(ReadArmJoints(1)); 
        distY = abs(ReadArmJoints(2));
        if distX <= 0.1 && distY <= 0.1
            error_grip = robot_arm.open_hand(); 
            [error_gripStatus, gripStatus] = robot_arm.get_hand_state();
            pause(1);
            if exit == 1
                stop = 1; 
            end
            if gripStatus == 0
                goToStart = 0;
                goToTarget = 1;
            end
        end

    elseif graspObj == 1
        disp('GRASP OBJ');
        error_grip = robot_arm.close_hand();    %close
        [error_gripStatus, gripStatus] = robot_arm.get_hand_state();
        % graspObj = 0;
        % goToCheckpoint = 1;
        if gripStatus == 1
            wait_count = wait_count + 1;
            if(wait_count == 10)
                wait_count == 0;
                graspObj = 0;
                goToCheckpoint = 1;
            end
        end

    elseif releaseObj == 1
        disp('RELEASE OBJ');
        error_grip = robot_arm.open_hand();    %open
        [error_gripStatus, gripStatus] = robot_arm.get_hand_state();
        pause(1);
        if gripStatus == 0
            releaseObj = 0;
            goToStart = 1;
            exit = 1;
        end

    end

    

    if error == 1 || error_grip == 1 || error_gripStatus == 1
        sim.terminate();
        return;
    end 
    
    m=m+1;
    %----------------------------------------------------------------------
    %It allows to guarantee a minimum cycle time of 50ms for the 
    %computation cycle in Matlab
    time = toc(start);
    if time<0.05
        pause(0.05-time);
    else
        pause(0.01);
    end
    start = tic;
    %----------------------------------------------------------------------
end
error = sim.terminate();