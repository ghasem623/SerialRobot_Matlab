classdef IK_NR_SYM < handle
    
    properties
        robot
        list_locked_q
        locked_q
        list_active_q
        max_IK_iteration
        epsilon
    end
    methods
        
        function obj = IK_NR_SYM(robot,list_locked_q,locked_q,max_IK_iteration,epsilon)
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
            
            
            
            list_theta=sym('theta',[1 obj.robot.n]);
            list_d=sym('d',[1 obj.robot.n]);
            row_rev=find(obj.robot.MatDH(:,1)==0);
            row_per=find(obj.robot.MatDH(:,1)==1);
            
            cur_MatDH=list_theta(1)*ones(obj.robot.n,5);
            
            cur_MatDH(row_rev,5)=list_theta(row_rev);
            cur_MatDH(row_per,5)=obj.robot.MatDH(row_per,5);
            vect_var(row_rev,1)=list_theta(row_rev);
            
            cur_MatDH(row_per,4)=list_theta(row_per);
            cur_MatDH(row_rev,4)=obj.robot.MatDH(row_rev,4);
            vect_var(row_per,1)=list_theta(row_per);

            cur_MatDH(:,3)=obj.robot.MatDH(:,3);
            cur_MatDH(:,2)=obj.robot.MatDH(:,2);
            cur_MatDH(:,1)=obj.robot.MatDH(:,1);
            
            
            
            
            mat_rot=eye(3);
            v_r=[0;0;0];
            
            for itdof=1:obj.robot.n
                
                cur_v_r=v_r+mat_rot*[0;0;cur_MatDH(itdof,4)];
                
                theta=cur_MatDH(itdof,5);alpha=cur_MatDH(itdof,3);
                cur_mat_rot=[cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta);  sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta); 0 sin(alpha) cos(alpha)];
                mat_rot=mat_rot*cur_mat_rot;
                v_r=cur_v_r+mat_rot*[cur_MatDH(itdof,2);0;0];
                
            end

            target_v_r(1:3,1)=sym('r',[1 3]);
            target_mat_rot(1:3,1)=sym('n',[1 3]);
            target_mat_rot(1:3,2)=sym('o',[1 3]);
            target_mat_rot(1:3,3)=sym('a',[1 3]);
            
            
%             error_t=v_r-target_v_r;
%             error_r=mat_rot-target_mat_rot;
%             
%             matlabFunction(error_t,'File','fun_error_t','Vars',[vect_var.',target_v_r.',target_mat_rot(1:3,1).',target_mat_rot(1:3,2).',target_mat_rot(1:3,3).']);
%             matlabFunction(error_r,'File','fun_error_r','Vars',[vect_var.',target_v_r.',target_mat_rot(1:3,1).',target_mat_rot(1:3,2).',target_mat_rot(1:3,3).']);
%           
            
            
            f_T=[mat_rot(1:3,1).'*(target_v_r-v_r); mat_rot(1:3,2).'*(target_v_r-v_r); mat_rot(1:3,3).'*(target_v_r-v_r)];
            f_R=[mat_rot(1:3,3).'*target_mat_rot(1:3,2)-target_mat_rot(1:3,3).'*mat_rot(1:3,2); mat_rot(1:3,1).'*target_mat_rot(1:3,3)-target_mat_rot(1:3,1).'*mat_rot(1:3,3); mat_rot(1:3,2).'*target_mat_rot(1:3,1)-target_mat_rot(1:3,2).'*mat_rot(1:3,1)]/2;
            sym_vect_f=[f_T;f_R];
            sym_mat_jacob=jacobian(sym_vect_f,vect_var);         
            matlabFunction(sym_vect_f,'File','fun_error','Vars',[vect_var.',target_v_r.',target_mat_rot(1:3,1).',target_mat_rot(1:3,2).',target_mat_rot(1:3,3).']);  
            matlabFunction(sym_mat_jacob,'File','jacobian_error','Vars',[vect_var.',target_v_r.',target_mat_rot(1:3,1).',target_mat_rot(1:3,2).',target_mat_rot(1:3,3).']);
        end
        
        function q_t= compute(obj,target_v_r,target_mat_rot,q0)
            cur_q=q0(1:length(obj.list_active_q));
            cur_q_t=zeros(obj.robot.n,1);
            cur_q_t(obj.list_active_q,1)=q0(1:length(obj.list_active_q));
            cur_q_t(obj.list_locked_q,1)=obj.locked_q;
            for it=1:obj.max_IK_iteration
                error_input=num2cell([cur_q_t',target_v_r',target_mat_rot(:,1)',target_mat_rot(:,2)',target_mat_rot(:,3)']);
                cur_error=fun_error(error_input{:});
               
                if norm(cur_error)<obj.epsilon
                    break
                else
                    cur_J= jacobian_error(error_input{:});
                    cur_J(:,obj.locked_q)=[];
                    cur_q=cur_q-pinv(cur_J)*cur_error;
                    cur_q_t(obj.list_active_q)=cur_q;
                end
            end
            q_t=cur_q_t;
        end
    end
end








