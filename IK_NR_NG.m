classdef IK_NR_NG < handle
    
    properties
        model
    end
    methods
        
        function obj = IK_NR_NG(varargin)
            obj.model=varargin{1};
        end
        
        function q= Compute(robot, x_ee, q0)
            
            
        end
        
    end  
end