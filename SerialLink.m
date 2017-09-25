


classdef SerialLink < handle
    
    properties
        links
        base_r
        base_rot
        n
        MatDH
    end
    methods
        
        function r = SerialLink(varargin)
            r.links = varargin;
            r.MatDH=[varargin{1}.jointtype; varargin{1}.a; varargin{1}.alpha; varargin{1}.d; varargin{1}.theta]';
            r.n=length(varargin{1});
            
        end
        
        function [v_r,mat_rot] = Pose_EE(robot, q, varargin)
       
            if length(q) ~= robot.n
                error('q must have %d columns', robot.n);
            end
            row_rev=(robot.MatDH(:,1)==0);
            row_per=(robot.MatDH(:,1)==1);
            robot.MatDH(row_rev,5)=q(row_rev);
            robot.MatDH(row_per,4)=q(row_per);
            
            
            mat_rot=eye(3);
            v_r=[0;0;0];
            
            for itdof=1:robot.n
                
                cur_v_r=v_r+mat_rot*[0;0;robot.MatDH(itdof,4)];
                
                theta=robot.MatDH(itdof,5);alpha=robot.MatDH(itdof,3);
                cur_mat_rot=[cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta);  sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta); 0 sin(alpha) cos(alpha)];
                mat_rot=mat_rot*cur_mat_rot;
                v_r=cur_v_r+mat_rot*[robot.MatDH(itdof,2);0;0];
                
            end            
        end
        
        function RobotPlot(robot, q, varargin)            
            if length(q) ~= robot.n
                error('q must have %d columns', robot.n);
            end
            axisrate=1;
            if nargin>0
                axisrate=varargin{1};
            end
            row_rev=(robot.MatDH(:,1)==0);
            row_per=(robot.MatDH(:,1)==1);
            robot.MatDH(row_rev,5)=q(row_rev);
            robot.MatDH(row_per,4)=q(row_per); 
            carleng=sum(robot.MatDH(:,2));
            jcoor=[0 0;0 0;-0.3*carleng/robot.n 0.3*carleng/robot.n];
            mat_rot=eye(3);
            v_r=[0;0;0]; 
            for itdof=1:robot.n
                cur_jcoor=v_r*ones(1,2)+mat_rot*jcoor;
                plot3(cur_jcoor(1,:),cur_jcoor(2,:),cur_jcoor(3,:),'LineWidth',5)
                hold on
                cur_v_r=v_r+mat_rot*[0;0;robot.MatDH(itdof,4)];
                plot3([v_r(1,1) cur_v_r(1,1)],[v_r(2,1) cur_v_r(2,1)],[v_r(3,1) cur_v_r(3,1)],'LineWidth',2)
                axis([-axisrate*carleng axisrate*carleng -axisrate*carleng axisrate*carleng -axisrate*carleng axisrate*carleng]);                
                theta=robot.MatDH(itdof,5);alpha=robot.MatDH(itdof,3);
                cur_mat_rot=[cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta);  sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta); 0 sin(alpha) cos(alpha)];
                mat_rot=mat_rot*cur_mat_rot;
                v_r=cur_v_r+mat_rot*[robot.MatDH(itdof,2);0;0];      
                plot3([cur_v_r(1,1) v_r(1,1)],[cur_v_r(2,1) v_r(2,1)],[cur_v_r(3,1) v_r(3,1)],'LineWidth',2)

            end           
        end        
    end
end











