%   arm_interface.m 
%   Interface to a simulated robot in CoppeliaSim simulator
%   2022/01/12
%   % Copyright (C) 2022
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef arm_interface < handle   
    properties (Access = private)
        vrep
        clientID 
        
        RobotHandle  % handle for the robot object
        jointHandle  % 1,2,... handle for each of the joints of the manipulator
        HandHandle   % handle for the hand object (BarrrettHand)
    end
    
    properties
        number_of_joints  % variable indicating the number of joints of the manipulator arm
        robot_name        % variable indicating the name of the handler in CoppeliaSim
        hand_name         % variable indicating the name of the handles in CoppeliaSim
        MinJointPos       % vector indicating the minimum position allowed for the joint i
        MaxJointPos       % vector indicating the maximun position allowed for the joint i
        
    end
    
    methods
        function [obj,error] = arm_interface(sim_obj,robot_name,hand_name)
            obj.robot_name = robot_name;
            obj.hand_name = hand_name;
            error = 0;
            
            [obj.vrep, obj.clientID] = sim_obj.get_connection();
            
            if obj.clientID <= -1
                clear obj;
                msg = 'ERROR: sim_obj seems to have an invalid connection to simulator!\n';
                error(msg)
            end
            
            %% Get objects handles
            
            obj.robot_name = robot_name;
            %Get Arm Handle
            [res, obj.RobotHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, robot_name, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting robot handle');
                error = 1;
                return;
            end
            
            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end
            
            
            %joint handles
            stop = 0;
            j=1;
            while stop==0
                jointName{j} = [robot_name,'_joint',num2str(j)];
                
                [res,obj.jointHandle{j}]=obj.vrep.simxGetObjectHandle(obj.clientID,jointName{j},obj.vrep.simx_opmode_blocking);
                
                if (res ~= obj.vrep.simx_return_ok)
                    stop=1;
                    obj.number_of_joints = j-1;
                    obj.jointHandle{:,j} = [];
                    jointName{:,j} = [];
                    %msg = 'ERROR: Failed getting sensor handle ';
                    %error(msg);
                else
                    obj.vrep.simxSetJointTargetVelocity (obj.clientID,obj.jointHandle{j},0*pi/180,obj.vrep.simx_opmode_oneshot);
                    
                    [res,~]=obj.vrep.simxGetJointPosition(obj.clientID, obj.jointHandle{j},obj.vrep.simx_opmode_streaming);
                    
                    if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                        disp('ERROR: Failed getting joint information');
                        error = 1;
                        return;
                    end
                end
                
                j=j+1;
                
            end
        end
        
        %Function that allows you to get the arm characteristics
        function [error,nJoints,Link,DistanceHand,MinPositionJoint,MaxPositionJoint] = get_RobotCharacteristics(obj)
            error = 0;
            nJoints = obj.number_of_joints;
            name = obj.robot_name;

            if strcmp(name,'MTB_2DOF')==1
                MinPositionJoint(1)=-160*pi/180;
                MinPositionJoint(2)=-120*pi/180;
                MaxPositionJoint(1)=160*pi/180;
                MaxPositionJoint(2)=120*pi/180;
                Link(1)=0.475-0.0;
                Link(2)=0.875-0.475;
            elseif strcmp(name,'MTB_3DOF')==1
                MinPositionJoint(1)=-160*pi/180;
                MinPositionJoint(2)=-160*pi/180;
                MinPositionJoint(3)=-160*pi/180;
                MaxPositionJoint(1)=160*pi/180;
                MaxPositionJoint(2)=160*pi/180;
                MaxPositionJoint(3)=160*pi/180;
                Link(1)=0.475-0.0;
                Link(2)=0.875-0.475;    %0.4m
                Link(3)=1.275-0.875;    %0.4m
            else
                disp('ERROR: No arm defined');
                error = 1;
                return;
            end

            DistanceHand = 0.0755+0.018;
            %Palma da mão 0.0755

            obj.MinJointPos = MinPositionJoint;
            obj.MaxJointPos = MaxPositionJoint;
        end
        
        %Function that allows you to get the position of the arm
        function [error,armPosition]=get_robot_position(obj)
            error = 0;
            [res,armPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.RobotHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end
        end
        
        %Function that allows you to set the joints from the arm
        function [error] = set_joints(obj, armJoints) %armJoints (1-6) in rad
            error = 0;
            for i=1:1:obj.number_of_joints
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)
                    disp('ERROR: value outside the limits of the joints');
                    error=1;
                    return;
                else
                    %res = obj.vrep.simxSetJointPosition(obj.clientID, obj.jointHandle{i},armJoints(i), obj.vrep.simx_opmode_oneshot);
                    res = obj.vrep.simxSetJointTargetPosition(obj.clientID, obj.jointHandle{i},armJoints(i), obj.vrep.simx_opmode_oneshot);
                    if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                        disp('ERROR: Failed sending Joint value!');
                        error=1;
                        return;
                    end
                end
            end
        end
        
        %Function that allows you to get the joints values from the arm
        function [error,armJoints] = get_joints(obj)
            % Get pose
            % x and y in cm
            % phi in rad
            error = 0;
            for i = 1:obj.number_of_joints
                %Read from joints
                [res,armJoints(i)]=obj.vrep.simxGetJointPosition(obj.clientID, obj.jointHandle{i},obj.vrep.simx_opmode_oneshot );
                if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                    disp('ERROR: Failed read Joint value!');
                    error = 1;
                    return;
                end
            end
        end
        
        %Functions that allows close the hand
        function error = close_hand(obj)
            error = 0;
            value = 1;      %Close Hand = 1  
            [res] = obj.vrep.simxSetIntegerSignal(obj.clientID, 'BarrettHandClose', value, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed close hand!');
                error=1;
                return;
            end
        end
        
        %Functions that allows open the hand
        function error = open_hand(obj)
            error = 0;
            value = 0;  %Open Hand = 0
            [res] = obj.vrep.simxSetIntegerSignal(obj.clientID, 'BarrettHandClose', value, obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed open hand!');
                error = 1;
                return;
            end
        end

        %Function that returns the state of the hand
        %state = 0 -> open
        %state = 1 -> close
        function [error,state] = get_hand_state(obj)
            error = 0;
            if strcmp(obj.hand_name,'RG2')==1
                [res,value] = obj.vrep.simxGetIntegerSignal(obj.clientID, 'RG2_open', obj.vrep.simx_opmode_blocking); %Open Hand = 1
                value = not(value); %Lógica contrária
            else
                [res,value] = obj.vrep.simxGetIntegerSignal(obj.clientID, 'BarrettHandClose', obj.vrep.simx_opmode_blocking); %Open Hand = 0
            end
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed get hand state!');
                error = 1;
                return;
            else
                state = value;
            end
        end
    end
end

