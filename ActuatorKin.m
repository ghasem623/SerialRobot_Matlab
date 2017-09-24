classdef ActuatorKin < handle
    
    properties
        
        q
        dq
        ddq
        num_sample
        num_motor
    end
    methods
        
        function s = ActuatorKin(varargin)
            if nargin>0
                s.q = varargin{1};
            end
            if nargin>1
                s.dq = varargin{2};
            end
            if nargin>2
                s.ddq = varargin{3};
            end
            [s.num_sample,s.num_motor]=size(s.q);           
        end
    end
    
    
end