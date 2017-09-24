



classdef Jacobian < handle
    
    properties
        actuatorkin
        robot
        Je_total
        J0_total
    end
    methods
        
        function s = Jacobian(robot,actuatorkin)
            if (~isa(robot,'SerialLink'))||(~isa(actuatorkin,'ActuatorKin'))
                error('The model should be of SerialLink Class');
            elseif robot.n~=actuatorkin.num_motor
                error('Number of robot DoF is not equal to number of motor input');
            else
                s.robot = robot;
                s.actuatorkin = actuatorkin;
            end
        end
        
        
        
        function compute(obj)
            for itSample=1:obj.actuatorkin.num_sample
                
                caur_mat_rot=eye(3);
                cur_v_a=zeros(3,1);
                cur_Je=[];
                
                
                for itdof=obj.robot.n:-1:1
                    
                    
                    cur_v_a=cur_v_a+caur_mat_rot'*[obj.robot.MatDH(itdof,2);0;0];
                    alpha=obj.robot.MatDH(itdof,3);
                    
                    
                    if obj.robot.MatDH(itdof,1)==0
                        theta=obj.actuatorkin.q(itSample,itdof);
                        mat_rot_i=[cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta);  sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta); 0 sin(alpha) cos(alpha)];
                        caur_mat_rot=mat_rot_i*caur_mat_rot;
                        cur_v_a=cur_v_a+caur_mat_rot'*[0;0;obj.robot.MatDH(itdof,4)];
                        cure=caur_mat_rot'*[0;0;1];
                        curcolumn=[cross(cure,cur_v_a);cure];
                    else
                        theta=obj.robot.MatDH(itdof,5);
                        mat_rot_i=[cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta);  sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta); 0 sin(alpha) cos(alpha)];
                        caur_mat_rot=mat_rot_i*caur_mat_rot;
                        cur_v_a=cur_v_a+caur_mat_rot'*[0;0;obj.actuatorkin.q(itSample,itdof)];
                        cure=caur_mat_rot'*[0;0;1];
                        curcolumn=[cure;zeros(3,1)];
                    end
                    cur_Je=[curcolumn,cur_Je]; 
                    
                    
                end
                cur_J0=[caur_mat_rot zeros(3,3); zeros(3,3) caur_mat_rot]*cur_Je;
                obj.Je_total(:,:,itSample)=cur_Je; 
                obj.J0_total(:,:,itSample)=cur_J0; 
            end    
        end 
    end
end






