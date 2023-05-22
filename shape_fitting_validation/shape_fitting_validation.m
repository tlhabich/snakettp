%First validation of shape fitting
%% Params
close all; clear; clc;
params=load_params();
params.task_prio_gain=1;
%Load functions
fkine = @fkine_num_mex;
jgeom=@jgeom_num_mex;
%Parameter for validation
num_shapes = 100;
shape_gains=[0.75,1,1.25];
%Plot colors
imesblau   = [0 80 155 ]/255; 
imesorange = [231 123 41 ]/255; 
imesgruen  = [200 211 23 ]/255;
%% Preallocation
% Number of shapes to generate
results.num_shapes = num_shapes;
% Different shape gains
results.shape_gains=shape_gains;
% Number of joints
results.n_joints = params.n_joints;
% Randomly generated desired joint positions
results.q_rand = nan(params.n_joints,num_shapes,length(shape_gains));
% Resulting joint vector
results.frechet_q_3t0r = nan(params.n_joints,num_shapes,length(shape_gains));   
results.frechet_q_3t2r = nan(params.n_joints,num_shapes,length(shape_gains));   
results.frechet_q_3t3r = nan(params.n_joints,num_shapes,length(shape_gains));
results.point2_q_3t0r = nan(params.n_joints,num_shapes,length(shape_gains));   
results.point2_q_3t2r = nan(params.n_joints,num_shapes,length(shape_gains));   
results.point2_q_3t3r = nan(params.n_joints,num_shapes,length(shape_gains));
results.point4_q_3t0r = nan(params.n_joints,num_shapes,length(shape_gains));   
results.point4_q_3t2r = nan(params.n_joints,num_shapes,length(shape_gains));   
results.point4_q_3t3r = nan(params.n_joints,num_shapes,length(shape_gains));
% Frechet distance (convergence)
results.frechet_3t0r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_3t2r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_3t3r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.point2_3t0r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.point2_3t2r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.point2_3t3r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.point4_3t0r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.point4_3t2r = zeros(num_shapes,params.max_iterations,length(shape_gains));
results.point4_3t3r = zeros(num_shapes,params.max_iterations,length(shape_gains));
% EE pose error (convergence);
results.frechet_pose_3T_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_2R_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_3R_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_3T_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_2R_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_3R_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_3T_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_2R_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.frechet_pose_3R_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_3T_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_2R_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_3R_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_3T_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_2R_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_3R_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_3T_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_2R_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point2_pose_3R_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_3T_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_2R_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_3R_error_3t0r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_3T_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_2R_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_3R_error_3t2r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_3T_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_2R_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
results.point4_pose_3R_error_3t3r = nan(num_shapes,params.max_iterations,length(shape_gains));
% Mean time
results.time_history=nan(1,num_shapes*length(shape_gains));
% Start configuration for shape fitting
q0 = zeros(params.n_joints,1);
% link positions
P_d = nan(3,params.n_joints);
frechet_dist=@DiscreteFrechetDistance;
%% Main
time_idx=1;
for i_shape = 1:num_shapes
    fprintf("Evaluating shape %d of %d\n",i_shape,num_shapes);
    % random joint vector as desired shape
    q_rand = (params.qmin*ones(params.n_joints,1) + (2*params.qmax)*rand(params.n_joints,1))*0.5;
    q_rand(1)=0;%feeder position zero
    results.q_rand(:,i_shape) = q_rand;
    % desired link positions
    [T_0_EE_des,P_d]=fkine(q_rand,params.n_joints,params.h);
    % generate a desired shape as spline 
    spline_struct = cscvn([[0;0;0],P_d]);
    %points = generate_points_from_path(spline_struct, params.n_joints, lengths, 0.01);
    for i_gain=1:length(shape_gains)
        %% Shape fitting (3T+Frechet)
        n_tasks=2;
        eq_values=zeros(4,params.n_joints,n_tasks);
        eq_gains=zeros(1,n_tasks);
        eq_params=zeros(1,n_tasks);
        eq_types=zeros(1,n_tasks);
        %EE
        eq_params(1)=params.n_joints;
        eq_types(1)=1;%3T
        eq_values(1:4,1:4,1)=T_0_EE_des;
        eq_gains(1)=params.task_prio_gain;
        max_dim_eq=3;
        dim_eq=3;
        %Frechet
        eq_params(2)=params.n_joints;
        eq_types(2)=6;%Frechet
        eq_values(1:3,:,2)=P_d;
        eq_gains(2)=params.task_prio_gain*shape_gains(i_gain);
        dim_eq=dim_eq+1;
        eq.params = eq_params;
        eq.types = eq_types;
        %Solve IK
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, ...
            params.h,params.delta_t,params.max_delta_q,1,params.max_delta_qF,1, ...
            [-params.qmax,params.qmax],fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
    %     %Debug
    %     [T_0_EE_temp,P_temp]=fkine(q_des,params.n_joints,params.h);
    %     plot3(P_d(1,:),P_d(2,:),P_d(3,:),'Color',imesgruen);hold on
    %     plot3(P_temp(1,:),P_temp(2,:),P_temp(3,:),'Color',imesblau);
        %axis equal
        %Save errors
        t1=mean(time);
        results.frechet_3t0r(i_shape,:,i_gain)=frechet_conv;
        results.frechet_pose_3T_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.frechet_pose_2R_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.frechet_pose_3R_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.frechet_q_3t0r(:,i_shape,i_gain)=q_des;
        %% Shape fitting (3T2R+Frechet)
        eq_types(1)=3;%3T2R
        max_dim_eq=5;
        dim_eq=5;
        %Frechet
        dim_eq=dim_eq+1;
        eq.types = eq_types;
        %Solve IK
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, ...
            params.h,params.delta_t,params.max_delta_q,1,params.max_delta_qF,1, ...
            [-params.qmax,params.qmax],fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
    %     %Debug
    %     figure;
    %     [T_0_EE_temp,P_temp]=fkine(q_des,params.n_joints,params.h);
    %     plot3(P_d(1,:),P_d(2,:),P_d(3,:),'x','Color',imesgruen);hold on
    %     plot3(P_temp(1,:),P_temp(2,:),P_temp(3,:),'Color',imesblau);
    %     axis equal
        %Save errors
        t2=mean(time);
        results.frechet_3t2r(i_shape,:,i_gain)=frechet_conv;
        results.frechet_pose_3T_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.frechet_pose_2R_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.frechet_pose_3R_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.frechet_q_3t2r(:,i_shape,i_gain)=q_des;
        %% Shape fitting (3T3R+Frechet)
        eq_types(1)=4;%3T3R
        max_dim_eq=6;
        dim_eq=6;
        %Frechet
        dim_eq=dim_eq+1;
        eq.types = eq_types;
        %Solve IK
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, ...
            params.h,params.delta_t,params.max_delta_q,1,params.max_delta_qF,1, ...
            [-params.qmax,params.qmax],fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
    %     %Debug
    %     figure;
    %     [T_0_EE_temp,P_temp]=fkine(q_des,params.n_joints,params.h);
    %     plot3(P_d(1,:),P_d(2,:),P_d(3,:),'x','Color',imesgruen);hold on
    %     plot3(P_temp(1,:),P_temp(2,:),P_temp(3,:),'Color',imesblau);
    %     axis equal
        %Save errors
        t3=mean(time);
        results.frechet_3t3r(i_shape,:,i_gain)=frechet_conv;
        results.frechet_pose_3T_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.frechet_pose_2R_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.frechet_pose_3R_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));        
        results.frechet_q_3t3r(:,i_shape,i_gain)=q_des;

        %% Shape fitting (3T+Point+n_s=2)
        n_tasks=length(params.n_joints-1:-2:1);
        eq_values=zeros(4,4,n_tasks);
        eq_gains=zeros(1,n_tasks);
        eq_params=zeros(1,n_tasks);
        eq_types=zeros(1,n_tasks);
        %EE 3T
        eq_params(1)=params.n_joints;
        eq_types(1)=1;
        eq_values(:,:,1)=T_0_EE_des;
        eq_gains(1)=params.task_prio_gain;
        max_dim_eq=3;
        dim_eq=max_dim_eq;
        count_point=2;
        T_eye=eye(4);
        for i=params.n_joints-1:-2:1
            if i==params.n_joints-1
                %skip
            else
                eq_params(count_point)=i;
                eq_types(count_point)=2;% euclidean distance task for all actuators (except ee)
                T_eye(1:3,4)=P_d(:,i);
                eq_values(:,:,count_point)=T_eye;
                eq_gains(count_point)=params.task_prio_gain*shape_gains(i_gain);
                count_point=count_point+1;
                dim_eq=dim_eq+1;
            end
        end
        eq.params = eq_params;
        eq.types = eq_types;
    
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
            params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
            fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
        
        %Save errors
        t4=mean(time);
        results.point2_3t0r(i_shape,:,i_gain)=frechet_conv;
        results.point2_pose_3T_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.point2_pose_2R_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.point2_pose_3R_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.point2_q_3t0r(:,i_shape,i_gain)=q_des;

        %% Shape fitting (3T2R+Point+n_s=2)
        %EE 3T2R
        eq_types(1)=3;%3T2R
        max_dim_eq=5;
        dim_eq=dim_eq+2;
        eq.types = eq_types;
    
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
            params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
            fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
        
        %Save errors
        t5=mean(time);
        results.point2_3t2r(i_shape,:,i_gain)=frechet_conv;
        results.point2_pose_3T_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.point2_pose_2R_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.point2_pose_3R_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.point2_q_3t2r(:,i_shape,i_gain)=q_des;

        %% Shape fitting (3T3R+Point+n_s=2)
        %EE 3T3R
        eq_types(1)=4;%3T3R
        max_dim_eq=6;
        dim_eq=dim_eq+1;
        eq.types = eq_types;
    
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
            params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
            fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
        
        %Save errors
        t6=mean(time);
        results.point2_3t3r(i_shape,:,i_gain)=frechet_conv;
        results.point2_pose_3T_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.point2_pose_2R_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.point2_pose_3R_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.point2_q_3t3r(:,i_shape,i_gain)=q_des;

        %% Shape fitting (3T+Point+n_s=4)
        n_tasks=length(params.n_joints-1:-4:1);
        eq_values=zeros(4,4,n_tasks);
        eq_gains=zeros(1,n_tasks);
        eq_params=zeros(1,n_tasks);
        eq_types=zeros(1,n_tasks);
        %EE 3T
        eq_params(1)=params.n_joints;
        eq_types(1)=1;
        eq_values(:,:,1)=T_0_EE_des;
        eq_gains(1)=params.task_prio_gain;
        max_dim_eq=3;
        dim_eq=max_dim_eq;
        count_point=2;
        T_eye=eye(4);
        for i=params.n_joints-1:-4:1
            if i==params.n_joints-1
                %skip
            else
                eq_params(count_point)=i;
                eq_types(count_point)=2;% euclidean distance task for all actuators (except ee)
                T_eye(1:3,4)=P_d(:,i);
                eq_values(:,:,count_point)=T_eye;
                eq_gains(count_point)=params.task_prio_gain*shape_gains(i_gain);
                count_point=count_point+1;
                dim_eq=dim_eq+1;
            end
        end
        eq.params = eq_params;
        eq.types = eq_types;
    
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
            params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
            fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
        
        %Save errors
        t7=mean(time);
        results.point4_3t0r(i_shape,:,i_gain)=frechet_conv;
        results.point4_pose_3T_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.point4_pose_2R_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.point4_pose_3R_error_3t0r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.point4_q_3t0r(:,i_shape,i_gain)=q_des;

        %% Shape fitting (3T2R+Point+n_s=4)
        %EE 3T2R
        eq_types(1)=3;%3T2R
        max_dim_eq=5;
        dim_eq=dim_eq+2;
        eq.types = eq_types;
    
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
            params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
            fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
        
        %Save errors
        t8=mean(time);
        results.point4_3t2r(i_shape,:,i_gain)=frechet_conv;
        results.point4_pose_3T_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.point4_pose_2R_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.point4_pose_3R_error_3t2r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.point4_q_3t2r(:,i_shape,i_gain)=q_des;

        %% Shape fitting (3T3R+Point+n_s=4)
        %EE 3T3R
        eq_types(1)=4;%3T3R
        max_dim_eq=6;
        dim_eq=dim_eq+1;
        eq.types = eq_types;
    
        [q_des,frechet_conv,pose_conv,time] = shape_fitting(params.n_joints, ...
            max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
            params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
            fkine,jgeom,q0,eye(4),1,frechet_dist,P_d);
        
        %Save errors
        t9=mean(time);
        results.point4_3t3r(i_shape,:,i_gain)=frechet_conv;
        results.point4_pose_3T_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(1:3,:));
        results.point4_pose_2R_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(4:5,:));
        results.point4_pose_3R_error_3t3r(i_shape,:,i_gain)=vecnorm(pose_conv(4:6,:));
        results.point4_q_3t3r(:,i_shape,i_gain)=q_des;

        %Mean time
        mean_time=[t1,t2,t3,t4,t5,t6,t7,t8,t9];
        results.time_history(time_idx)=mean(mean_time);
        time_idx=time_idx+1;
    end
end 

%% Plot frechet results
close all
%plot settings
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'DefaultLineLineWidth',2);
set(groot,'defaultAxesFontSize',14)
%set(gcf,'PaperPositionMode','auto');

fig=figure;
fig.Position(3:4)=[550,600];
s1=subplot(2,2,1);
%Frechet convergence
plot(1:params.max_iterations,mean(results.frechet_3t0r(:,:,1))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_3t0r(:,:,2))/(params.h*10^3),'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_3t0r(:,:,3))/(params.h*10^3),':','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_3t2r(:,:,1))/(params.h*10^3),'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_3t2r(:,:,2))/(params.h*10^3),'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_3t2r(:,:,3))/(params.h*10^3),':','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_3t3r(:,:,1))/(params.h*10^3),'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_3t3r(:,:,2))/(params.h*10^3),'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_3t3r(:,:,3))/(params.h*10^3),':','Color',imesgruen);
set(gca,'xticklabel',{[]})
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
%ylim([1,5]);
ylabel("$\bar{\mathcal{X}}_\mathrm{F}$")
xlabel("");
grid on;
set(gca, 'YScale', 'log');

%3T convergence
s2=subplot(2,2,2);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t0r(:,:,1))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t0r(:,:,2))/(params.h*10^3),'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t0r(:,:,3))/(params.h*10^3),':','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t2r(:,:,1))/(params.h*10^3),'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t2r(:,:,2))/(params.h*10^3),'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t2r(:,:,3))/(params.h*10^3),':','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t3r(:,:,1))/(params.h*10^3),'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t3r(:,:,2))/(params.h*10^3),'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t3r(:,:,3))/(params.h*10^3),':','Color',imesgruen);
set(gca,'xticklabel',{[]})
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{3T}$")
xlabel("");
leg=legend('3T0R $\lambda='+string(shape_gains(1))+'$','3T0R $\lambda='+string(shape_gains(2))+'$', ...
    '3T0R $\lambda='+string(shape_gains(3))+'$',...
    '3T2R $\lambda='+string(shape_gains(1))+'$','3T2R $\lambda='+string(shape_gains(2))+'$', ...
    '3T2R $\lambda='+string(shape_gains(3))+'$', ...
    '3T3R $\lambda='+string(shape_gains(1))+'$','3T3R $\lambda='+string(shape_gains(2))+'$', ...
    '3T3R $\lambda='+string(shape_gains(3))+'$','NumColumns',3,'Location','northoutside');
grid on;

%2R convergence
s3=subplot(2,2,3);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t0r(:,:,1))*180/pi,'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t0r(:,:,2))*180/pi,'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t0r(:,:,3))*180/pi,':','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t2r(:,:,1))*180/pi,'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t2r(:,:,2))*180/pi,'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t2r(:,:,3))*180/pi,':','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t3r(:,:,1))*180/pi,'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t3r(:,:,2))*180/pi,'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t3r(:,:,3))*180/pi,':','Color',imesgruen);

set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{2R}$ in deg")
xlabel("Iteration")
grid on;

%3R convergence
s4=subplot(2,2,4);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t0r(:,:,1))*180/pi,'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t0r(:,:,2))*180/pi,'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t0r(:,:,3))*180/pi,':','Color',imesorange);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t2r(:,:,1))*180/pi,'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t2r(:,:,2))*180/pi,'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t2r(:,:,3))*180/pi,':','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t3r(:,:,1))*180/pi,'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t3r(:,:,2))*180/pi,'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t3r(:,:,3))*180/pi,':','Color',imesgruen);

set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{3R}$ in deg")
xlabel("Iteration")
grid on;
s1.Position([4])=s2.Position([4]);
s3.Position([4])=s2.Position([4]);
s4.Position([4])=s2.Position([4]);
s3.Position([2])=s2.Position([2])-1.1*s2.Position([4]);
s4.Position([2])=s2.Position([2])-1.1*s2.Position([4]);
movegui(fig,'west')

%% Plot point results
fig_point=figure;
fig_point.Position(3:4)=[550,600];
s1=subplot(2,2,1);
%Frechet convergence
plot(1:params.max_iterations,mean(results.point2_3t0r(:,:,1))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.point2_3t0r(:,:,2))/(params.h*10^3),'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_3t0r(:,:,3))/(params.h*10^3),':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_3t2r(:,:,1))/(params.h*10^3),'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_3t2r(:,:,2))/(params.h*10^3),'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_3t2r(:,:,3))/(params.h*10^3),':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_3t3r(:,:,1))/(params.h*10^3),'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_3t3r(:,:,2))/(params.h*10^3),'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_3t3r(:,:,3))/(params.h*10^3),':','Color',imesgruen);
set(gca,'xticklabel',{[]})
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{F}$")
xlabel("");
grid on;
%ylim([1,5]);
set(gca, 'YScale', 'log');

%3T convergence
s2=subplot(2,2,2);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t0r(:,:,1))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t0r(:,:,2))/(params.h*10^3),'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t0r(:,:,3))/(params.h*10^3),':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t2r(:,:,1))/(params.h*10^3),'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t2r(:,:,2))/(params.h*10^3),'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t2r(:,:,3))/(params.h*10^3),':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t3r(:,:,1))/(params.h*10^3),'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t3r(:,:,2))/(params.h*10^3),'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t3r(:,:,3))/(params.h*10^3),':','Color',imesgruen);
set(gca,'xticklabel',{[]})
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{3T}$")
xlabel("");
leg=legend('3T0R $\lambda='+string(shape_gains(1))+'$','3T0R $\lambda='+string(shape_gains(2))+'$', ...
    '3T0R $\lambda='+string(shape_gains(3))+'$', ...
    '3T2R $\lambda='+string(shape_gains(1))+'$','3T2R $\lambda='+string(shape_gains(2))+'$', ...
    '3T2R $\lambda='+string(shape_gains(3))+'$',...
    '3T3R $\lambda='+string(shape_gains(1))+'$','3T3R $\lambda='+string(shape_gains(2))+'$', ...
    '3T3R $\lambda='+string(shape_gains(3))+'$','NumColumns',3,'Location','northoutside');
grid on;

%2R convergence
s3=subplot(2,2,3);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t0r(:,:,1))*180/pi,'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t0r(:,:,2))*180/pi,'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t0r(:,:,3))*180/pi,':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t2r(:,:,1))*180/pi,'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t2r(:,:,2))*180/pi,'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t2r(:,:,3))*180/pi,':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t3r(:,:,1))*180/pi,'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t3r(:,:,2))*180/pi,'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t3r(:,:,3))*180/pi,':','Color',imesgruen);

set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{2R}$ in deg")
xlabel("Iteration")
grid on;

%3R convergence
s4=subplot(2,2,4);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t0r(:,:,1))*180/pi,'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t0r(:,:,2))*180/pi,'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t0r(:,:,3))*180/pi,':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t2r(:,:,1))*180/pi,'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t2r(:,:,2))*180/pi,'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t2r(:,:,3))*180/pi,':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t3r(:,:,1))*180/pi,'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t3r(:,:,2))*180/pi,'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t3r(:,:,3))*180/pi,':','Color',imesgruen);

set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{3R}$ in deg")
xlabel("Iteration")
grid on;
s1.Position([4])=s2.Position([4]);
s3.Position([4])=s2.Position([4]);
s4.Position([4])=s2.Position([4]);
s3.Position([2])=s2.Position([2])-1.1*s2.Position([4]);
s4.Position([2])=s2.Position([2])-1.1*s2.Position([4]);
movegui(fig_point,'east')

%% Plot for paper (frechet vs. point)
fig_paper=figure;
fig_paper.Position(3:4)=[550,550];
s1=subplot(2,2,1);
i_gain=2;
%Frechet convergence
plot(1:params.max_iterations,mean(results.frechet_3t0r(:,:,i_gain))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_3t2r(:,:,i_gain))/(params.h*10^3),'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_3t3r(:,:,i_gain))/(params.h*10^3),'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_3t0r(:,:,i_gain))/(params.h*10^3),'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_3t2r(:,:,i_gain))/(params.h*10^3),'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_3t3r(:,:,i_gain))/(params.h*10^3),'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point4_3t0r(:,:,i_gain))/(params.h*10^3),':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point4_3t2r(:,:,i_gain))/(params.h*10^3),':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point4_3t3r(:,:,i_gain))/(params.h*10^3),':','Color',imesgruen);
set(gca,'xticklabel',{[]})
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{d}_\mathrm{F}/h$");
xlabel("");
grid on;
ylim([.7,5]);
set(gca, 'YScale', 'log');

%3T convergence
s2=subplot(2,2,2);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t0r(:,:,i_gain))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t2r(:,:,i_gain))/(params.h*10^3),'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3T_error_3t3r(:,:,i_gain))/(params.h*10^3),'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t0r(:,:,i_gain))/(params.h*10^3),'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t2r(:,:,i_gain))/(params.h*10^3),'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3T_error_3t3r(:,:,i_gain))/(params.h*10^3),'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point4_pose_3T_error_3t0r(:,:,i_gain))/(params.h*10^3),':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point4_pose_3T_error_3t2r(:,:,i_gain))/(params.h*10^3),':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point4_pose_3T_error_3t3r(:,:,i_gain))/(params.h*10^3),':','Color',imesgruen);
set(gca,'xticklabel',{[]})
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{3T}/h$")
xlabel("");
leg=legend("3T0R Fr\'echet","3T2R Fr\'echet","3T3R Fr\'echet", ...
    '3T0R Point $n_\mathrm{s}=2$','3T2R Point $n_\mathrm{s}=2$','3T3R Point $n_\mathrm{s}=2$', ...
    '3T0R Point $n_\mathrm{s}=4$','3T2R Point $n_\mathrm{s}=4$','3T3R Point $n_\mathrm{s}=4$', ...
    'NumColumns',3,'Location','northoutside');
grid on;

%2R convergence
s3=subplot(2,2,3);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t0r(:,:,i_gain))*180/pi,'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t2r(:,:,i_gain))*180/pi,'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_2R_error_3t3r(:,:,i_gain))*180/pi,'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t0r(:,:,i_gain))*180/pi,'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t2r(:,:,i_gain))*180/pi,'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_2R_error_3t3r(:,:,i_gain))*180/pi,'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point4_pose_2R_error_3t0r(:,:,i_gain))*180/pi,':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point4_pose_2R_error_3t2r(:,:,i_gain))*180/pi,':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point4_pose_2R_error_3t3r(:,:,i_gain))*180/pi,':','Color',imesgruen);
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{2R}$ in deg")
xlabel("Iteration")
grid on;

%3R convergence
s4=subplot(2,2,4);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t0r(:,:,i_gain))*180/pi,'-','Color',imesorange);
hold on;
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t2r(:,:,i_gain))*180/pi,'-','Color',imesblau);
plot(1:params.max_iterations,mean(results.frechet_pose_3R_error_3t3r(:,:,i_gain))*180/pi,'-','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t0r(:,:,i_gain))*180/pi,'--','Color',imesorange);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t2r(:,:,i_gain))*180/pi,'--','Color',imesblau);
plot(1:params.max_iterations,mean(results.point2_pose_3R_error_3t3r(:,:,i_gain))*180/pi,'--','Color',imesgruen);
plot(1:params.max_iterations,mean(results.point4_pose_3R_error_3t0r(:,:,i_gain))*180/pi,':','Color',imesorange);
plot(1:params.max_iterations,mean(results.point4_pose_3R_error_3t2r(:,:,i_gain))*180/pi,':','Color',imesblau);
plot(1:params.max_iterations,mean(results.point4_pose_3R_error_3t3r(:,:,i_gain))*180/pi,':','Color',imesgruen);
set(gca,'TickLabelInterpreter','latex');
xlim([1,params.max_iterations]);
ylabel("$\bar{\mathcal{X}}_\mathrm{3R}$ in deg")
xlabel("Iteration")
grid on;
s1.Position([4])=s2.Position([4]);
s3.Position([4])=s2.Position([4]);
s4.Position([4])=s2.Position([4]);
s3.Position([2])=s2.Position([2])-1.1*s2.Position([4]);
s4.Position([2])=s2.Position([2])-1.1*s2.Position([4]);
movegui(fig_paper,'east')
