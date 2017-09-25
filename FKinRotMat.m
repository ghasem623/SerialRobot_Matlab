
classdef FKinRotMat < FKinAnalysis
    
    properties
        
        mat_rot_total
        
    end
    methods
        
        function s = FKinRotMat(robot)
            s@FKinAnalysis(robot);
        end
        
        function compute(obj,q_input)
            [nrow,num_sample]=size(q_input);
            for itSample=1:num_sample
                [v_r,mat_rot]=obj.robot.Pose_EE(q_input(:,itSample));              
                obj.v_r_total(:,itSample)=v_r;
                obj.mat_rot_total(:,:,itSample)=mat_rot;
            end
            
        end
        
    end
    
    
end
