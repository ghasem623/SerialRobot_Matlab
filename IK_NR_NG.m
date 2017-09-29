classdef IK_NR_NG < handle
    
    properties
        robot
        list_locked_q
        locked_q
        list_active_q
        max_IK_iteration
        epsilon
    end
    methods
        
        function obj = IK_NR_NG(robot,list_locked_q,locked_q,max_IK_iteration,epsilon)
            obj.robot=robot;
            if nargin<2
                if robot.n>6
                    obj.list_locked_q=linspace(1,robot.n-6,robot.n-6);
                    obj.locked_q=zeros(1,robot.n-6);
                else
                    obj.list_locked_q=[];
                    obj.locked_q=[];
                end
                obj.max_IK_iteration=20;
                obj.epsilon=1e-8;
            elseif nargin<3
                obj.list_locked_q=list_locked_q;
                obj.locked_q=zeros(1,length(list_locked_q));
                obj.max_IK_iteration=20;
                obj.epsilon=1e-8;
            elseif nargin<4
                obj.list_locked_q=list_locked_q;
                obj.locked_q=locked_q;
                obj.max_IK_iteration=20;
                obj.epsilon=1e-8;
            elseif nargin<5
                obj.list_locked_q=list_locked_q;
                obj.locked_q=locked_q;
                obj.max_IK_iteration=max_IK_iteration;
                obj.epsilon=1e-8;
            else
                obj.list_locked_q=list_locked_q;
                obj.locked_q=locked_q;
                obj.max_IK_iteration=max_IK_iteration;
                obj.epsilon=epsilon;
            end
            obj.list_active_q=setxor(linspace(1,robot.n,robot.n),obj.list_locked_q);   
        end
        
        function q_t= compute(obj,target_v_r,target_mat_rot,q0)
            cur_q=q0(1:length(obj.list_active_q));
            cur_q_t=zeros(obj.robot.n,1);
            cur_q_t(obj.list_active_q,1)=q0(1:length(obj.list_active_q));
            cur_q_t(obj.list_locked_q,1)=obj.locked_q;
            for it=1:obj.max_IK_iteration            
                [cur_v_r,cur_mat_rot] = obj.robot.Pose_EE(cur_q_t);
                cur_vect_R=vect(cur_mat_rot);
                cur_tr_R=trace(cur_mat_rot);
                f_R=2*(cur_vect_R-vect(target_mat_rot));
                f_S=cur_tr_R-trace(target_mat_rot);
                f_T=cur_v_r-target_v_r;
                vect_f=[f_R;f_S;f_T];
                if norm(vect_f)<obj.epsilon
                    break 
                else                    
                    cur_K_obj=Jacobian(obj.robot);
                    cur_K_obj.compute(cur_q_t);
                    curK0=cur_K_obj.J0_total;
                    curK0(:,obj.locked_q)=[];
                    cur_J=[(eye(3)*cur_tr_R-cur_mat_rot)*curK0(4:6,:); -2*(cur_vect_R')*curK0(4:6,:);curK0(1:3,:)];                  
                    cur_q=cur_q-pinv(cur_J)*vect_f;
                    cur_q_t(obj.list_active_q)=cur_q;
                end
            end 
            q_t=cur_q_t;
        end
    end
end








