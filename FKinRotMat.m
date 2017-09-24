
classdef FKinRotMat < FKinAnalysis
    
    properties
        
        mat_rot_total
        
    end
    methods
        
        function s = FKinRotMat(robot,actuatorkin)
            s@FKinAnalysis(robot,actuatorkin);
        end
        
        function compute(obj)
            for itSample=1:obj.actuatorkin.num_sample
                [v_r,mat_rot]=obj.robot.Pose_EE(obj.actuatorkin.q(itSample,:));              
                obj.v_r_total(:,itSample)=v_r;
                obj.mat_rot_total(:,:,itSample)=mat_rot;
            end
            
        end
        
    end
    
    
end
