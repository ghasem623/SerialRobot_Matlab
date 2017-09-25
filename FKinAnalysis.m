

classdef FKinAnalysis < handle
    
    properties
        robot
        v_r_total
    end
    methods
        
        function s = FKinAnalysis(robot)
            if (~isa(robot,'SerialLink'))
                error('The model should be of SerialLink Class');

            else
                s.robot = robot;
            end
        end
   end
end