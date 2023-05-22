%Shape fitting using task-priority inverse kinematics
%Input:
%n_act_joints double
%   Number of active joints (starting at the tip)
%max_dim_eq double
%   Maximum dimension of equality tasks
%dim_eq double
%   Sum of all equality task dimensions
%eq struct
%   Struct of all equality tasks consisting of task_param and task_type 
%   (see set_based_task_priority_ik repository:
%   functions/compute_J_sigma_tilde_eq.m)
%eq_values double [4 x 4 x k]
%   Desired pose value for link i for all equality tasks k
%eq_gains double [1 x k]
%   Gain for all equality tasks k
%n_joints double
%   Number of all joints
%n_iter double
%   Number of CLIK iterations
%h double
%   Actuator height
%delta_t double
%   Time step for numerical integration
%max_delta_q double
%   Maximum joint angle change per CLIK iteration
%enable_q_scale logical
%   Enable scaling of joint angles to max_delta_q
%max_delta_q1 double
%   Maximum feeder position change per CLIK iteration
%enable_q1_scale logical
%   Enable scaling of feeder to max_delta_q1
%q_limit double [1 x 2]
%   Angle limits of all joints [qmin qmax]   
%fkine_handle function handle
%   Forward kinematics to all links i 
%jgeom_handle function handle
%   Geometric jacobian to all links i
%q double [n x 1]
%   Current joint angles
%T_N_E double [4 x 4]
%   Static transformation EE-flange -> tool tip (if not necessary -> eye(4))
%enable_data_saving logical
%   Enable saving of convergence data
%frechet_dist_handle function handle
%   Calculate discrete frechet distance (points columnwise)
%P_d [3 x n_joints]
%   Desired shape to calculate frechet distance in every iteration
%Output:
%q_des double [n_joints x 1]
%   Shape fitting solution
%frechet_conv [n_iter x 1]
%   Shape fitting convergence (history of frechet error)
%pose_conv [6 x n_iter]
%   Pose convergence (history of 3T3R error)
function [q_des,frechet_conv,pose_conv,time_per_iteration] = shape_fitting(n_act_joints, ...
    max_dim_eq,dim_eq,eq,eq_values,eq_gains, ...
    n_joints,n_iter,h,delta_t,max_delta_q,enable_q_scale,max_delta_q1, ...
    enable_q1_scale,q_limit,fkine_handle,jgeom_handle,q,T_N_E, ...
    enable_data_saving,frechet_dist_handle,P_d)

    J_eq=zeros(max_dim_eq,n_joints,length(eq.types)); % maximum possible size
    sigma_tilde=zeros(max_dim_eq,length(eq.types)); % maximum possible size
    J_aug=zeros(dim_eq,n_joints);
    delta_q=zeros(length(q),1);
    dim_task_all=zeros(size(eq_values,3),1);
    time_per_iteration=zeros(n_iter,1);
    q_des=zeros(length(q),1);
    frechet_calc=0;
    if enable_data_saving
        frechet_conv=zeros(n_iter,1);
        pose_conv=zeros(6,n_iter);
    end
    for iter = 1:n_iter % ik iterations
        tic;
        % compute all jacobians for equality tasks 
        % (one time per ik iteration with updated q!)
        for k_current = 1:size(eq_values,3)
            [J,sigma_tilde_out,dim_task] = compute_J_sigma_tilde_eq(eq.types(k_current),...
                eq_values(:,:,k_current),eq.params(k_current),fkine_handle,jgeom_handle,q, ...
                h,T_N_E,frechet_dist_handle,n_act_joints);
            sigma_tilde(1:dim_task,k_current)=sigma_tilde_out(1:dim_task,:);
            J_eq(1:dim_task,1:eq.params(k_current),k_current)=J(1:dim_task,:);
            dim_task_all(k_current)=dim_task;
            if eq.types(k_current)==6%Frechet distance
                frechet_temp=-sigma_tilde_out(1:dim_task,:);%sigma_tilde=0-sigma
                frechet_calc=1;
            end
        end
        
        joint_clamping_flag=0;
        joint_state=zeros(1,n_joints);%1=active 0=inactive/clamped on limit
        joint_state(1)=1; %feeder always active
        joint_state(end:-1:end-n_act_joints+1)=1;
        while ~joint_clamping_flag
            % get indices of active joints
            i_act_joints=find(joint_state==1);
            % compute IK step ONLY with active joints
            q_dot=zeros(length(i_act_joints),1);
            line_J_aug=0;
            for k_current = 1:size(eq_values,3)
                if k_current==1
                    % N=eye(DoF) for FIRST equality task
                    N=eye(length(i_act_joints));
                else
                    % compute null space projector N=eye-pinv(J_aug)*J_aug
                    N=eye(length(i_act_joints))-pinv(J_aug(1:line_J_aug,i_act_joints))*J_aug(1:line_J_aug,i_act_joints);
                end
                % add joint velocity from equality task #k_current
                q_dot=q_dot+N*pinv(J_eq(1:dim_task_all(k_current),i_act_joints,k_current))*eye(dim_task_all(k_current))*eq_gains(k_current)*sigma_tilde(1:dim_task_all(k_current),k_current);
                % build J_aug for equality task #k_current+1
                J_aug(line_J_aug+1:line_J_aug+dim_task_all(k_current),:)=J_eq(1:dim_task_all(k_current),:,k_current);
                line_J_aug=line_J_aug+dim_task_all(k_current);
            end
            
            % numerical integration of q_dot
            % maximum delta_q with choosen time step
            delta_q=q_dot*delta_t;
                   
            % scale rotary actuators & feeder
            max_delta_q_current=max(abs(delta_q(2:end)));
            if max_delta_q_current>max_delta_q && enable_q_scale
                % scale
                delta_q=delta_q*max_delta_q/max_delta_q_current;
            end
            if delta_q(1)>max_delta_q1 && enable_q1_scale
                % scale
                delta_q=delta_q*max_delta_q1/delta_q(1);
            end
            
            % check joint angle limits
            joint_clamping_flag=1;
            temp_q_next=q;
            temp_q_next(i_act_joints)=q(i_act_joints)+delta_q;
            for i=1:length(i_act_joints)
                if i==1
                    % skip feeder
                elseif temp_q_next(i_act_joints(i))>q_limit(2)  
                    % joint limit violation -> set q(i) on qmax
                    q(i_act_joints(i))=q_limit(2);
                    joint_clamping_flag=0;
                    joint_state(i_act_joints(i))=0;%joint i clamped on limit for iteration
                    break;
                elseif temp_q_next(i_act_joints(i))<q_limit(1)
                    % joint limit violation -> set q(i) on qmin
                    q(i_act_joints(i))=q_limit(1);
                    joint_clamping_flag=0;
                    joint_state(i_act_joints(i))=0;%joint i clamped on limit for iteration
                    break;
                end
            end
            
            if joint_clamping_flag
                % q within joint limits -> update q
                q=temp_q_next;
            end
        end
    temp=toc;
    time_per_iteration(iter)=temp*10^3;%ms
    if enable_data_saving
        if frechet_calc
            frechet_conv(iter)=frechet_temp*10^3;%mm
        else
            [~,P_current]=fkine_handle(q,n_joints,h);
            frechet_temp=frechet_dist_handle(P_d,P_current);
            frechet_conv(iter)=frechet_temp*10^3;%mm
        end
        pose_error=zeros(6,1);
        pose_error=compute_3T2R_sigma_tilde(q,eq_values(1:3,:,1),eye(4),uint8(2), n_joints, false, fkine_handle,h);
        pose_error(1:3)=pose_error(1:3)*10^3;%mm
        pose_conv(:,iter)=pose_error;
    end
    end
    q_des=q;
    mean(time_per_iteration);
end