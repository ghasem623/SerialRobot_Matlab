

classdef FKinAnalysis < handle
    
    properties
        robot
        actuatorkin
        v_r_total
    end
    methods
        
        function s = FKinAnalysis(robot,actuatorkin)
            if (~isa(robot,'SerialLink'))||(~isa(actuatorkin,'ActuatorKin'))
                error('The model should be of SerialLink Class');
            elseif robot.n~=actuatorkin.num_motor
                error('Number of robot DoF is not equal to number of motor input');
            else
                s.robot = robot;
                s.actuatorkin = actuatorkin;              
            end
        end
   end
end