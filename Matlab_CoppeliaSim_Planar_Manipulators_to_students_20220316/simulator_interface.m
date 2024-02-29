%   simulator_interface.m 
%   Interface to simulator in CoppeliaSim simulator
%   2022/01/12
%   % Copyright (C) 2022
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef simulator_interface < handle
   
    properties (Access = private)
        vrep
        clientID
        TargetHandle        % handle for the target object RED_BOX
        ObstacleHandle      % handle for the obstacle
    end
    
    properties
        TARGET_Number       % Number of targets
    end
    
    methods
        % Create a connection handler to a SAMU Simulation scene
        % remote_ip_address The remote IP address
        % remote_port The remote port
        function [obj,error] = simulator_interface(remote_ip_address, remote_port)
            error = 0;
            ip_address = '127.0.0.1';
            port = 19997;
            if nargin >= 1
                % TODO: Add safe guarding for well formated ip
                ip_address = remote_ip_address;
            end
            if nargin >= 2
                % TODO: Check for valid number
                port = remote_port;
            end
            
            obj.vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.vrep.simxFinish(-1); % just in case, close all opened connections
            
            obj.clientID = obj.vrep.simxStart(ip_address, port, true, true, 5000, 5);
            
            if (obj.clientID > -1)
                fprintf('\n');
                disp('INFO: Connected successfully to CoppeliaSim!');
                fprintf('\n');
                
            else
                clear obj;
                disp('ERROR: Failed connecting to remote API server. Ensure CoppeliaSim is running and a scenario is loaded.');
                error = 1;
                return;
            end
            
            %% Synchronous mode
            obj.vrep.simxSynchronous(obj.clientID, true); % Enable the synchronous mode
            obj.vrep.simxStartSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            
            obj.vrep.simxSynchronousTrigger(obj.clientID); % Trigger next simulation step (Blocking function call)
            
            % The first simulation step is now being executed
            %ensure it is finished so we can access signals
            obj.vrep.simxGetPingTime(obj.clientID);
            
            % Eight different boxs
            TARGET_NUMBER = 1;
            
            %Get Targets Handle
            [res, obj.TargetHandle{1}] = obj.vrep.simxGetObjectHandle(obj.clientID, 'RedBox', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting target handle');
                error = 1;
                return;
            end   
     
            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{1},-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting object position information');
                error=1;
                return;
            end

            %Get Obstacle Handle
            [res, obj.ObstacleHandle] = obj.vrep.simxGetObjectHandle(obj.clientID, 'Obstacle', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok)
                disp('ERROR: Failed getting obstacle handle');
                error = 1;
                return;
            end   
     
            [res,~] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.ObstacleHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting object position information');
                error=1;
                return;
            end
            
            
            
            %% Setup data streaming
            % simulation time
            [res, ~] = obj.vrep.simxGetFloatSignal(obj.clientID,'SimulationTime', obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed to setup data streaming for simulation time');
                error = 1;
                return;
            end
        end
        
        function [vrep, clientID] = get_connection(obj)
            vrep = obj.vrep;
            clientID = obj.clientID;
        end
        
        % call before setting velocity and getting data
        function ensure_all_data(obj)
            obj.vrep.simxGetPingTime(obj.clientID);
        end
        
        % call just after all data have been collected.
        function trigger_simulation(obj)
            obj.vrep.simxSynchronousTrigger(obj.clientID);
        end
        
        %Function that allows you to get the position of the target
        function [error,targetPosition]=get_target_position(obj)
            iTarget=1;
            error = 0;
            [res,targetPosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.TargetHandle{iTarget},-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end
        end

        %Function that allows you to get the position of the obstacle
        function [error,obstaclePosition]=get_obstacle_position(obj)
            error = 0;
            [res,obstaclePosition] = obj.vrep.simxGetObjectPosition(obj.clientID,obj.ObstacleHandle,-1,obj.vrep.simx_opmode_streaming);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed getting robot position information');
                error = 1;
                return;
            end
        end
        
        function [error,sim_time] = get_simulation_time(obj)
            error = 0;
            [res, sim_time] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTime', obj.vrep.simx_opmode_buffer);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: The simulation stopped in CoppeliaSim');
                error = 1;
                return;
            end
        end
        
        function [error,sim_timestep] = get_simulation_timestep(obj)
            error = 0;
            [res, sim_timestep] = obj.vrep.simxGetFloatSignal(obj.clientID, 'SimulationTimeStep', obj.vrep.simx_opmode_blocking);
            if (res ~= obj.vrep.simx_return_ok )
                disp('ERROR: Failed getting simulation time step!');
                error = 1;
                return;
            end
        end
        
        function error = terminate(obj)
            error = 0;
            res = obj.vrep.simxStopSimulation(obj.clientID, obj.vrep.simx_opmode_oneshot);
            if (res ~= obj.vrep.simx_return_ok && res ~= obj.vrep.simx_return_novalue_flag)
                disp('ERROR: Failed stopping simulation!');
                error = 1;
                return;
            end
            obj.vrep.simxGetPingTime(obj.clientID);
            
            % Now close the connection to CoppeliaSim:
            obj.vrep.simxFinish(obj.clientID);
            
            obj.vrep.delete(); % call the destructor!
        end
    end
    
end

