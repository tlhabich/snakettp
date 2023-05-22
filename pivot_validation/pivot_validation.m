%Changing the pointing direction at a constant position by means of
%reconfiguration (Pivot motion with EE as pivot point)
%% Init
close all; clear; clc;
result_names=["env1.mat";"env2.mat";"env3.mat"];
env_names=["sph_pos_env1.mat";"sph_pos_env2.mat";"sph_pos_env3.mat"];
params=load_params();
params.n_point_corr=4;
T_N_E=troty(pi/2);
%Load functions
fkine = @fkine_num_mex;
jgeom=@jgeom_num_mex;
%plot settings
imesblau   = [0 80 155 ]/255; 
imesorange = [231 123 41 ]/255; 
imesgruen  = [200 211 23 ]/255;
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'DefaultLineLineWidth',2);
set(groot,'defaultAxesFontSize',12)
set(gcf,'PaperPositionMode','auto');
theta_max=pi/3;
n_theta=10;%number of changes of theta
n_phi=10;%number of changes of phi
frechet_dist=@DiscreteFrechetDistance;
%% Preallocation
results.frechet_error=nan(n_phi,n_theta,length(result_names));
results.orth_dev_avg=nan(n_phi,n_theta,length(result_names));
results.orth_dev_max=nan(n_phi,n_theta,length(result_names));
results.frechet_error_without=nan(n_phi,n_theta,length(result_names));
results.orth_dev_avg_without=nan(n_phi,n_theta,length(result_names));
results.orth_dev_max_without=nan(n_phi,n_theta,length(result_names));
results.frechet_error_point=nan(n_phi,n_theta,length(result_names));
results.orth_dev_avg_point=nan(n_phi,n_theta,length(result_names));
results.orth_dev_max_point=nan(n_phi,n_theta,length(result_names));
%% Main
for i_env=1:length(result_names)
    %get sphere positions
    load("../tele_validation/mjx_env/"+env_names(i_env));
    sph_pos=sph_pos';
    sph_pos=[[0;0;30*params.h],[0;0;31*params.h],[0;0;32*params.h],sph_pos];
    spline_struct = cscvn(sph_pos);
    %get desired positions of all snake frames (from sphere positions)
    lengths = [0.5; ones(params.n_joints-2,1); 0.5]*params.h;
    P_d = generate_points_from_path(spline_struct, params.n_joints, lengths, 0.001);
    P_d=P_d';
    %% Compute initial fit
    n_tasks=2;
    eq_values=zeros(4,params.n_joints,n_tasks);
    eq_gains=zeros(1,n_tasks);
    eq_params=zeros(1,n_tasks);
    eq_types=zeros(1,n_tasks);
    eq_params(1)=params.n_joints;
    eq_types(1)=1;%3T to compute desired orientation
    T_eye=eye(4);
    T_eye(1:3,4)=P_d(:,end);
    eq_values(1:4,1:4,1)=T_eye;
    eq_gains(1)=params.task_prio_gain;
    max_dim_eq=3;
    dim_eq=3;
    eq_params(2)=params.n_joints;
    eq_types(2)=6;%Frechet
    eq_values(1:3,:,2)=P_d;
    eq_gains(2)=params.task_prio_gain;
    dim_eq=dim_eq+1;
    eq.params = eq_params;
    eq.types = eq_types;
    q0=ones(params.n_joints,1)*0.01;
    q0(1)=params.n_joints*params.h;
    [q_des] = shape_fitting(params.n_joints, ...
        max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
        params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
        fkine,jgeom,q0,T_N_E,0,frechet_dist,P_d);
    %compute error for shape fitting
    [orth_dev_avg, orth_dev_max,pos_current, pos_orth] = calc_orth_error_avg_max(spline_struct, q_des, params.h, params.n_joints, fkine);
    pos_current_init=pos_current;
%     %plot shape fitting solution
%     figure
%     plot3(P_d(1,:)/params.h,P_d(2,:)/params.h,P_d(3,:)/params.h,'o','Color','red');hold on
%     [~,pos_current]=fkine(q_des,params.n_joints,params.h);
%     pos_current=pos_current';
%     plot3(pos_current(:,1)/params.h,pos_current(:,2)/params.h,pos_current(:,3)/params.h,'-','Color',imesgruen);
%     axis equal;
%     view(-48.5468,63.8482);


    %% Pivot motion
    q_des_pivot_all=nan(params.n_joints,(n_phi+1)*(n_theta+1));
    q_des_pivot_all_without=nan(params.n_joints,(n_phi+1)*(n_theta+1));
    q_des_pivot_all_point=nan(params.n_joints,(n_phi+1)*(n_theta+1));
    T_0_EE_all=nan(4,4,(n_phi+1)*(n_theta+1));
    %z-Direction as pointing direction
    T_0_EE_init=fkine(q_des,31,params.h)*T_N_E;
    %visualize different pointing directions
    figure;
    trplot(T_0_EE_init,'color',imesorange);
    hold on
    i_theta=1;
    count=1;
    theta_phi_des=nan(2,(n_theta+1)*(n_phi+1));
    for theta=(0:n_theta)*theta_max/n_theta
        i_phi=1;
        for phi = (0:n_phi)*2*pi/n_phi
            %% Pivot EE 3T2R with shape fitting using frechet distance
            T_0_EE_temp=T_0_EE_init*calc_T_0_sphere(theta,phi);
            eq_values(1:4,1:4,1)=T_0_EE_temp;
            max_dim_eq=5;
            dim_eq=5+1;
            eq_types(1)=3;
            eq.types = eq_types;
            [q_des_pivot] = shape_fitting(params.n_joints, ...
                max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
                params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
                fkine,jgeom,q_des,T_N_E,0,frechet_dist,P_d);
            %compute error for shape fitting
            [orth_dev_avg, orth_dev_max,pos_current, pos_orth] = calc_orth_error_avg_max(spline_struct, q_des_pivot, params.h, params.n_joints, fkine);
            %compute error
            frechet_error=DiscreteFrechetDistance(P_d,pos_current');
            %visualize different pointing directions
            points=[T_0_EE_temp(1:3,4),T_0_EE_temp(1:3,4)+T_0_EE_temp(1:3,3)];
            plot3(points(1,:),points(2,:),points(3,:),'Color',imesgruen);

            %% Pivot EE 3T2R without shape fitting
            eq_params_without=[eq_params(1)];
            eq_types_without=[eq_types(1)];
            eq_values_without=eq_values(1:4,1:4,1);
            eq_gains_without=eq_gains(1);
            max_dim_eq_without=max_dim_eq;
            dim_eq_without=max_dim_eq;
            eq_without.types = eq_types_without;
            eq_without.params = eq_params_without;
            [q_des_pivot_without] = shape_fitting(params.n_joints, ...
                max_dim_eq_without,dim_eq_without,eq_without,eq_values_without,eq_gains_without,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
                params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
                fkine,jgeom,q_des,T_N_E,0,frechet_dist,P_d);
            %compute error for shape fitting
            [orth_dev_avg_without, orth_dev_max_without,pos_current_without, pos_orth_without] = calc_orth_error_avg_max(spline_struct, q_des_pivot_without, params.h, params.n_joints, fkine);
            %compute error
            frechet_error_without=DiscreteFrechetDistance(P_d,pos_current_without');

            %% Pivot EE 3T2R with shape fitting using point correspondences
            %compute shape fitting solution
            n_tasks=length(params.n_joints-1:-params.n_point_corr:1);
            eq_values_point=zeros(4,4,n_tasks);
            eq_gains_point=zeros(1,n_tasks);
            eq_params_point=zeros(1,n_tasks);
            eq_types_point=zeros(1,n_tasks);
            %EE 3T2R highest prio
            eq_params_point(1)=params.n_joints;
            eq_types_point(1)=3;
            eq_values_point(:,:,1)=T_0_EE_temp;
            eq_gains_point(1)=params.task_prio_gain;
            max_dim_eq_point=max_dim_eq;
            dim_eq_point=max_dim_eq;
            count_point=2;
            T_eye=eye(4);
            for i=params.n_joints-1:-params.n_point_corr:1
                if i==params.n_joints-1
                    %skip
                else
                    eq_params_point(count_point)=i;
                    eq_types_point(count_point)=2;% euclidean distance task for all actuators (except ee)
                    T_eye(1:3,4)=P_d(:,i);
                    eq_values_point(:,:,count_point)=T_eye;
                    eq_gains_point(count_point)=params.task_prio_gain;
                    count_point=count_point+1;
                    dim_eq_point=dim_eq_point+1;
                end
            end
            eq_point.params = eq_params_point;
            eq_point.types = eq_types_point;
%             q0=ones(params.n_joints,1)*0.01;
%             q0(1)=params.n_joints*params.h;
%             [q_des] = shape_fitting(params.n_joints, ...
%                 max_dim_eq,dim_eq,eq,eq_values,eq_gains,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
%                 params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax],fkine,jgeom,q0,T_N_E);
%             %compute error for shape fitting
%             [orth_dev_avg, orth_dev_max,pos_current, pos_orth] = calc_orth_error_avg_max(spline_struct, q_des, params.h, params.n_joints, fkine);
%             pos_shape_fitting_plot(:,:,i_env)=pos_current;
%             %plot shape fitting solution
%             fnplt(spline_struct,'-',0.75,'Color','black'); hold on
%             %plot3(P_d(:,1),P_d(:,2),P_d(:,3),'o','Color','red');
%             %plot3(pos_orth(:,1),pos_orth(:,2),pos_orth(:,3),'*','Color','red','MarkerSize',10,'LineWidth',1);
%             plot3(pos_current(:,1),pos_current(:,2),pos_current(:,3),'x','Color',imesblau,'MarkerSize',10,'LineWidth',1);
%             %legend('spline','P_d','pos_orth','pos_current');
%             trplot(fkine(q_des,31,params.h)*T_N_E,'color',imesblau,'length',2*params.h);
%             set(gca,'TickLabelInterpreter','latex');
        
            [q_des_pivot_point] = shape_fitting(params.n_joints, ...
                max_dim_eq_point,dim_eq_point,eq_point,eq_values_point,eq_gains_point,params.n_joints,params.max_iterations, params.h,params.delta_t, ...
                params.max_delta_q,1,params.max_delta_qF,1,[-params.qmax,params.qmax], ...
                fkine,jgeom,q_des,T_N_E,0,frechet_dist,P_d);
            %compute error for shape fitting
            [orth_dev_avg_point, orth_dev_max_point,pos_current_point, pos_orth_point] = calc_orth_error_avg_max(spline_struct, q_des_pivot_point, params.h, params.n_joints, fkine);
            frechet_error_point=DiscreteFrechetDistance(P_d,pos_current_point');

            %save (errors in mm)
            results.orth_dev_avg(i_phi,i_theta,i_env)=orth_dev_avg*10^3;
            results.orth_dev_max(i_phi,i_theta,i_env)=orth_dev_max*10^3;
            results.frechet_error(i_phi,i_theta,i_env)=frechet_error*10^3;
            results.orth_dev_avg_without(i_phi,i_theta,i_env)=orth_dev_avg_without*10^3;
            results.orth_dev_max_without(i_phi,i_theta,i_env)=orth_dev_max_without*10^3;
            results.frechet_error_without(i_phi,i_theta,i_env)=frechet_error_without*10^3;
            results.orth_dev_avg_point(i_phi,i_theta,i_env)=orth_dev_avg_point*10^3;
            results.orth_dev_max_point(i_phi,i_theta,i_env)=orth_dev_max_point*10^3;
            results.frechet_error_point(i_phi,i_theta,i_env)=frechet_error_point*10^3;
            T_0_EE_all(:,:,count)=T_0_EE_temp;
            q_des_pivot_all(:,count)=q_des_pivot;
            q_des_pivot_all_without(:,count)=q_des_pivot_without;
            q_des_pivot_all_point(:,count)=q_des_pivot_point;
            theta_phi_des(:,count)=[theta;phi];
            "Env "+i_env+", counter "+count
            count=count+1;
            i_phi=i_phi+1;
        end
        i_theta=i_theta+1;
    end
    
    %% Plot final shapes for cover
    count_plot=88;
    fig=figure;
    fig.Position(3:4)=[225,350];
    plot3(P_d(1,:)/params.h,P_d(2,:)/params.h,P_d(3,:)/params.h,'o','Color',imesgruen,"MarkerSize",3,"LineWidth",3);
    hold on;
    plot3(pos_current_init(:,1)/params.h,pos_current_init(:,2)/params.h,pos_current_init(:,3)/params.h,'-','Color',"black");
    %trplot(fkine(q_des,params.n_joints,params.h)*T_N_E,'color',imesorange,'length',2*params.h);
    %Initial pointing direction
    points=[T_0_EE_init(1:3,4),T_0_EE_init(1:3,4)+T_0_EE_init(1:3,3)*2*params.h]/params.h;
    dp=points(:,2)-points(:,1);
    quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
    dp(3)=dp(3)*1.5;
    ptext=points(:,1)+dp*1.4;
    text(ptext(1),ptext(2),ptext(3), '\boldmath$z_0$');
    %New pointing direction
    points=[T_0_EE_all(1:3,4,count_plot),T_0_EE_all(1:3,4,count_plot)+T_0_EE_all(1:3,3,count_plot)*2*params.h]/params.h;
    dp=points(:,2)-points(:,1);
    quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
    ptext=points(:,1)+dp*1.5;
    text(ptext(1),ptext(2),ptext(3), '\boldmath$z_\mathrm{d}$');
    [~,pos_current_all]=fkine(q_des_pivot_all(:,count_plot),params.n_joints,params.h);
    pos_current_all=pos_current_all';
    plot3(pos_current_all(:,1)/params.h,pos_current_all(:,2)/params.h,pos_current_all(:,3)/params.h,'-','Color',imesorange);
    [~,pos_current_all_point]=fkine(q_des_pivot_all_point(:,count_plot),params.n_joints,params.h);
    pos_current_all_point=pos_current_all_point';
    plot3(pos_current_all_point(:,1)/params.h,pos_current_all_point(:,2)/params.h,pos_current_all_point(:,3)/params.h,'-','Color',imesgruen);
    %trplot(T_0_EE_all(:,:,count_plot),'color',imesgruen,'length',2*params.h);
    %trplot(fkine(q_des_pivot_all(:,count_plot),params.n_joints,params.h)*T_N_E,'color',imesblau,'length',params.h);
    axis equal;
    grid on;
    xlabel('$x/h$');
    ylabel('$y/h$');
    zlabel('$z/h$');
    zlim([30,60]);
    if i_env==1
        view(37.4466,52.1758);
    elseif i_env==2
        view(-54.2248,68.0634);
    else
        view(-48.5468,63.8482);
    end
    leg=legend("Path",'Initial','','',"Contribution", ...
        "Reference", ...
        'NumColumns',2,'Location','eastoutside','Position',[0.0262    0.8041    0.9425    0.1226]);
    if i_env==3
        text(0,.75,24,'(b)');
    end
    %% Plot final shapes for validation chapter (path 3)
    if i_env==3
        count_left=(n_phi+1)*4-round(n_phi/5);
        count_right=(n_theta+1)*(n_phi+1)-round(n_phi/5);
        fig=figure;
        fig.Position(3:4)=[550,400];
        subplot(1,2,1);
        plot3(P_d(1,:)/params.h,P_d(2,:)/params.h,P_d(3,:)/params.h,':','Color',[.5,.5,.5]);
        hold on;
        plot3(pos_current_init(:,1)/params.h,pos_current_init(:,2)/params.h,pos_current_init(:,3)/params.h,'x','Color',imesorange,'MarkerSize',10,'LineWidth',1);
        %trplot(fkine(q_des,params.n_joints,params.h)*T_N_E,'color',imesorange,'length',2*params.h);
        %Initial pointing direction
        points=[T_0_EE_init(1:3,4),T_0_EE_init(1:3,4)+T_0_EE_init(1:3,3)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        dp(3)=dp(3)*1.5;
        ptext=points(:,1)+dp*1.2;
        text(ptext(1),ptext(2),ptext(3), '\boldmath$z_0$');
        %New pointing direction
        points=[T_0_EE_all(1:3,4,count_left),T_0_EE_all(1:3,4,count_left)+T_0_EE_all(1:3,3,count_left)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        ptext=points(:,1)+dp*1.3;
        text(ptext(1),ptext(2),ptext(3), '\boldmath$z_\mathrm{d}$');
        [~,pos_current_all]=fkine(q_des_pivot_all(:,count_left),params.n_joints,params.h);
        pos_current_all=pos_current_all';
        plot3(pos_current_all(:,1)/params.h,pos_current_all(:,2)/params.h,pos_current_all(:,3)/params.h,'-','Color',imesgruen);
        [~,pos_current_all_point]=fkine(q_des_pivot_all_point(:,count_left),params.n_joints,params.h);
        pos_current_all_point=pos_current_all_point';
        plot3(pos_current_all_point(:,1)/params.h,pos_current_all_point(:,2)/params.h,pos_current_all_point(:,3)/params.h,'-','Color',imesorange);
        [~,pos_current_all_without]=fkine(q_des_pivot_all_without(:,count_left),params.n_joints,params.h);
        pos_current_all_without=pos_current_all_without';
        plot3(pos_current_all_without(:,1)/params.h,pos_current_all_without(:,2)/params.h,pos_current_all_without(:,3)/params.h,'-','Color','black');
        %trplot(T_0_EE_all(:,:,count_left),'color',imesgruen,'length',2*params.h);
        %trplot(fkine(q_des_pivot_all(:,count_left),params.n_joints,params.h)*T_N_E,'color',imesblau,'length',params.h);
        axis equal;
        grid on;
        xlabel('$x/h$');
        ylabel('$y/h$');
        zlabel('$z/h$');
        zlim([30,60]);
        view(-48.5468,63.8482);
        text(0,.75,24,'(a)');
        subplot(1,2,2);
        plot3(P_d(1,:)/params.h,P_d(2,:)/params.h,P_d(3,:)/params.h,':','Color',[.5,.5,.5]);
        hold on;
        plot3(pos_current_init(:,1)/params.h,pos_current_init(:,2)/params.h,pos_current_init(:,3)/params.h,'x','Color',imesorange,'MarkerSize',10,'LineWidth',1);
        %trplot(fkine(q_des,params.n_joints,params.h)*T_N_E,'color',imesorange,'length',2*params.h);
        %Initial pointing direction
        points=[T_0_EE_init(1:3,4),T_0_EE_init(1:3,4)+T_0_EE_init(1:3,3)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        ptext=points(:,1)+dp*1.3;
        text(ptext(1),ptext(2),ptext(3), '\boldmath$z_0$');
        %New pointing direction
        points=[T_0_EE_all(1:3,4,count_right),T_0_EE_all(1:3,4,count_right)+T_0_EE_all(1:3,3,count_right)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        ptext=points(:,1)+dp*1.3;
        text(ptext(1),ptext(2),ptext(3), '\boldmath$z_\mathrm{d}$');
        [~,pos_current_all]=fkine(q_des_pivot_all(:,count_right),params.n_joints,params.h);
        pos_current_all=pos_current_all';
        plot3(pos_current_all(:,1)/params.h,pos_current_all(:,2)/params.h,pos_current_all(:,3)/params.h,'-','Color',imesgruen);
        [~,pos_current_all_point]=fkine(q_des_pivot_all_point(:,count_right),params.n_joints,params.h);
        pos_current_all_point=pos_current_all_point';
        plot3(pos_current_all_point(:,1)/params.h,pos_current_all_point(:,2)/params.h,pos_current_all_point(:,3)/params.h,'-','Color',imesorange);
        [~,pos_current_all_without]=fkine(q_des_pivot_all_without(:,count_right),params.n_joints,params.h);
        pos_current_all_without=pos_current_all_without';
        plot3(pos_current_all_without(:,1)/params.h,pos_current_all_without(:,2)/params.h,pos_current_all_without(:,3)/params.h,'-','Color','black');
        %trplot(T_0_EE_all(:,:,count_right),'color',imesgruen,'length',2*params.h);
        %trplot(fkine(q_des_pivot_all(:,count_right),params.n_joints,params.h)*T_N_E,'color',imesblau,'length',params.h);
        axis equal;
        grid on;
        xlabel('$x/h$');
        ylabel('$y/h$');
        zlabel('$z/h$');
        zlim([30,60]);
        view(-48.5468,63.8482);
        leg=legend("Path $\#$"+i_env,'Initial fit','','',"Fr\'echet", ...
            "Point","Off", ...
            'NumColumns',5,'Location','eastoutside','Position',[0.0429,0.9293,0.8624,0.0550]);
        text(0,.75,24,'(b)');
        %% Plot one final shape for validation chapter (path 3)
        count_right=(n_theta+1)*(n_phi+1)-round(n_phi/5);
        fig_path3=figure;
        fig_path3.Position(3:4)=[550,350];
        sub_path3=subplot(1,2,2);
        plot3(P_d(1,:)/params.h,P_d(2,:)/params.h,P_d(3,:)/params.h,'o','Color',imesgruen,"MarkerSize",2.5,"LineWidth",2.5);
        hold on;
        plot3(pos_current_init(:,1)/params.h,pos_current_init(:,2)/params.h,pos_current_init(:,3)/params.h,'-','Color','black');
        %trplot(fkine(q_des,params.n_joints,params.h)*T_N_E,'color',imesorange,'length',2*params.h);
        %Initial pointing direction
        points=[T_0_EE_init(1:3,4),T_0_EE_init(1:3,4)+T_0_EE_init(1:3,3)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        ptext=points(:,1)+dp*1.3;
        text(ptext(1),ptext(2),ptext(3), '\boldmath$z_0$');
        %New pointing direction
        points=[T_0_EE_all(1:3,4,count_right),T_0_EE_all(1:3,4,count_right)+T_0_EE_all(1:3,3,count_right)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        ptext=points(:,1)+dp*1.3;
        text(ptext(1),ptext(2),ptext(3), '\boldmath$z_\mathrm{d}$');
        [~,pos_current_all]=fkine(q_des_pivot_all(:,count_right),params.n_joints,params.h);
        pos_current_all=pos_current_all';
        plot3(pos_current_all(:,1)/params.h,pos_current_all(:,2)/params.h,pos_current_all(:,3)/params.h,'-','Color',imesorange);
        [~,pos_current_all_point]=fkine(q_des_pivot_all_point(:,count_right),params.n_joints,params.h);
        pos_current_all_point=pos_current_all_point';
        plot3(pos_current_all_point(:,1)/params.h,pos_current_all_point(:,2)/params.h,pos_current_all_point(:,3)/params.h,'-','Color',imesgruen);
        [~,pos_current_all_without]=fkine(q_des_pivot_all_without(:,count_right),params.n_joints,params.h);
        pos_current_all_without=pos_current_all_without';
        plot3(pos_current_all_without(:,1)/params.h,pos_current_all_without(:,2)/params.h,pos_current_all_without(:,3)/params.h,':','Color',[.5,.5,.5]);
        %trplot(T_0_EE_all(:,:,count_right),'color',imesgruen,'length',2*params.h);
        %trplot(fkine(q_des_pivot_all(:,count_right),params.n_joints,params.h)*T_N_E,'color',imesblau,'length',params.h);
        axis equal;
        grid on;
        xlabel('$x/h$');
        ylabel('$y/h$');
        zlabel('$z/h$');
        zlim([30,60]);
        view(-48.5468,63.8482);
        leg=legend("Path $\#$"+i_env,'Initial','','',"Fr\'echet", ...
            "Point","Off", ...
            'NumColumns',1,'FontSize',9,'Position',[0.8223    0.3799    0.1750    0.2226]);
        pos=get(sub_path3,'Position');
        set(sub_path3,'Position',[1.14*pos(1) 1.14*pos(2) pos(3) pos(4)]);
        %text(0,.75,24,'(b)');
        %saveas(fig,'C:\Users\root\Seafile\Eigene Veröffentlichungen\iros_2022_follow_the_leader\images\results_pivot_path3.fig','fig');
    end

    %% Plot of all final configurations for ICRA video
    for plot_count=12:(n_theta+1)*(n_phi+1)
        fig=figure;
        fig.Position(3:4)=[550,350];
        plot3(P_d(1,:)/params.h,P_d(2,:)/params.h,P_d(3,:)/params.h,'o','Color',imesgruen,"MarkerSize",2.5,"LineWidth",2.5);
        hold on;
        plot3(pos_current_init(:,1)/params.h,pos_current_init(:,2)/params.h,pos_current_init(:,3)/params.h,'-','Color','black');
        %trplot(fkine(q_des,params.n_joints,params.h)*T_N_E,'color',imesorange,'length',2*params.h);
        %Initial pointing direction
        points=[T_0_EE_init(1:3,4),T_0_EE_init(1:3,4)+T_0_EE_init(1:3,3)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        dp=dp*1.5;
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        %ptext=points(:,1)+dp*1.5;
        %text(ptext(1),ptext(2),ptext(3), '\boldmath$z_0$');
        %New pointing direction
        points=[T_0_EE_all(1:3,4,plot_count),T_0_EE_all(1:3,4,plot_count)+T_0_EE_all(1:3,3,plot_count)*2*params.h]/params.h;
        dp=points(:,2)-points(:,1);
        dp=dp*1.5;
        quiver3(points(1,1),points(2,1),points(3,1),dp(1),dp(2),dp(3),0,'LineWidth',2,'MaxHeadSize',100/norm(dp),'Color',imesblau);
        %ptext=points(:,1)+dp*1.5;
        %text(ptext(1),ptext(2),ptext(3), '\boldmath$z_\mathrm{d}$');
        [~,pos_current_all]=fkine(q_des_pivot_all(:,plot_count),params.n_joints,params.h);
        pos_current_all=pos_current_all';
        plot3(pos_current_all(:,1)/params.h,pos_current_all(:,2)/params.h,pos_current_all(:,3)/params.h,'-','Color',imesorange);
    %     [~,pos_current_all_point]=fkine(q_des_pivot_all_point(:,count_right),params.n_joints,params.h);
    %     pos_current_all_point=pos_current_all_point';
    %     plot3(pos_current_all_point(:,1)/params.h,pos_current_all_point(:,2)/params.h,pos_current_all_point(:,3)/params.h,'-','Color',imesgruen);
    %     [~,pos_current_all_without]=fkine(q_des_pivot_all_without(:,count_right),params.n_joints,params.h);
    %     pos_current_all_without=pos_current_all_without';
    %     plot3(pos_current_all_without(:,1)/params.h,pos_current_all_without(:,2)/params.h,pos_current_all_without(:,3)/params.h,':','Color',[.5,.5,.5]);
        %trplot(T_0_EE_all(:,:,count_right),'color',imesgruen,'length',2*params.h);
        %trplot(fkine(q_des_pivot_all(:,count_right),params.n_joints,params.h)*T_N_E,'color',imesblau,'length',params.h);
        axis equal;
        grid on;
        xlabel('$x/h$');
        ylabel('$y/h$');
        zlabel('$z/h$');
        zlim([30,60]);
        if i_env==1
            view(37.4466,52.1758);
            xlim([-.5,12.5]);
            ylim([-.5,12.5]);
        elseif i_env==2
            view(-54.2248,68.0634);
            xlim([-.5,12.5]);
            ylim([-1,10]);
        else
            view(-48.5468,63.8482);
            xlim([-.5,12.5]);
            ylim([-1,7.5]);
        end
        leg=legend("Path $\#$"+i_env,'Initial','','',"Fr\'echet", ...
        'NumColumns',3,'FontSize',9,'Position',[0.2530    0.8821    0.4883    0.0525]);
        %"Point","Off", ...
        %text(0,.75,24,'(b)');
        
        close(fig);
    end
    %%        
    %     % save results in mm
%     shape_fitting_frechet_error(1,i_env)=frechet_error*10^3;
%     shape_fitting_orth_dev_avg(1,i_env)=orth_dev_avg*10^3;
%     shape_fitting_orth_dev_max(1,i_env)=orth_dev_max*10^3;
%     %plot shape fitting
%     fnplt(spline_struct); hold on
%     plot3(P_d(:,1),P_d(:,2),P_d(:,3),'o');
%     plot3(pos_current(:,1),pos_current(:,2),pos_current(:,3),'x');

%     for i_folder=1:length(folders)
%         fprintf("Environment(%d) Folder(%d)\n",i_env,i_folder);
%         % get positions of all snake frames (at the end of telemanipulation)
%         current_result=load(folders(i_folder)+"\"+result_names(i_env));
%         current_q=current_result.out.q.signals.values(end,:);
%         
%         % calculate error
%         [orth_dev_avg, orth_dev_max,pos_current, pos_orth] = calc_orth_error_avg_max(spline_struct, current_q', params.h, params.n_joints, fkine);
%         frechet_error=DiscreteFrechetDistance(P_d',pos_current');
% 
%         % save final configuration
%         pos_current_plot(:,:,i_env)=pos_current;
% 
% %         fnplt(spline_struct); hold on
% %         plot3(P_d(:,1),P_d(:,2),P_d(:,3),'o');
% %         plot3(pos_current(:,1),pos_current(:,2),pos_current(:,3),'x');
% 
% %         plot orth_points
%         fnplt(spline_struct); hold on
%         %plot3(P_d(:,1),P_d(:,2),P_d(:,3),'o','Color','red');
%         plot3(pos_orth(:,1),pos_orth(:,2),pos_orth(:,3),'*','Color','red','MarkerSize',10,'LineWidth',1);
%         plot3(pos_current(:,1),pos_current(:,2),pos_current(:,3),'x','Color','blue','MarkerSize',10,'LineWidth',1);
%         legend('spline','pos_orth','pos_current');
% 
%         % save results in mm
%         results.frechet_error(i_folder,i_env)=frechet_error*10^3;
%         results.orth_dev_avg(i_folder,i_env)=orth_dev_avg*10^3;
%         results.orth_dev_max(i_folder,i_env)=orth_dev_max*10^3;
%     end
end

%% Subplot for validation chapter
sub1=subplot(1,2,1);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error(:,:,1))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error(:,:,2))/(params.h*10^3),'-','Color',imesblau);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error(:,:,3))/(params.h*10^3),'-','Color',imesgruen);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_point(:,:,1))/(params.h*10^3),':','Color',imesorange);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_point(:,:,2))/(params.h*10^3),':','Color',imesblau);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_point(:,:,3))/(params.h*10^3),':','Color',imesgruen);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_without(:,:,1))/(params.h*10^3),'--','Color',imesorange);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_without(:,:,2))/(params.h*10^3),'--','Color',imesblau);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_without(:,:,3))/(params.h*10^3),'--','Color',imesgruen);
set(gca,'TickLabelInterpreter','latex');
%s=scatter([1,2,3],shape_fitting_orth_dev_avg,'x','MarkerEdgeColor',imesgruen,'LineWidth',2);
%ylim([0,11]);
ylabel("$\bar{d}_\mathrm{F}/h$")
xlabel("$\theta$ in deg")
%title("avg error")
leg=legend('Fr\''echet $\#$1','Fr\''echet $\#$2','Fr\''echet $\#$3', ...
    'Point $\#$1','Point $\#$2','Point $\#$3','Off $\#$1','Off  $\#$2', ...
    'Off $\#$3','NumColumns',3,'FontSize',9,'Position',[0.0084    0.8147    0.5386    0.1205]);
grid on;
pos=get(sub1,'Position');
set(sub1,'Position',[0.6*pos(1) 1.8*pos(2) 1.4*pos(3) 0.7*pos(4)]);
text(28,-1.3,'(a)');
text(91,-1.3,'(b)');

%% Pivot results for validation chapter
fig_pivot=figure;
fig_pivot.Position(3:4)=[300,275];
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error(:,:,1))/(params.h*10^3),'-','Color',imesorange);
hold on;
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error(:,:,2))/(params.h*10^3),'-','Color',imesblau);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error(:,:,3))/(params.h*10^3),'-','Color',imesgruen);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_point(:,:,1))/(params.h*10^3),':','Color',imesorange);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_point(:,:,2))/(params.h*10^3),':','Color',imesblau);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_point(:,:,3))/(params.h*10^3),':','Color',imesgruen);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_without(:,:,1))/(params.h*10^3),'--','Color',imesorange);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_without(:,:,2))/(params.h*10^3),'--','Color',imesblau);
plot((0:n_theta)*theta_max/n_theta*180/pi,mean(results.frechet_error_without(:,:,3))/(params.h*10^3),'--','Color',imesgruen);
set(gca,'TickLabelInterpreter','latex');
%s=scatter([1,2,3],shape_fitting_orth_dev_avg,'x','MarkerEdgeColor',imesgruen,'LineWidth',2);
%ylim([0,11]);
ylabel("$\bar{d}_\mathrm{F}/h$")
xlabel("$\theta$ in deg")
xticks([0   15  30  45  60]);
xlim([0,60]);
%title("avg error")
leg=legend('Fr\''echet $\#$1','Fr\''echet $\#$2','Fr\''echet $\#$3', ...
    'Point $\#$1','Point $\#$2','Point $\#$3','Off $\#$1','Off  $\#$2', ...
    'Off $\#$3','NumColumns',3,'FontSize',9, ...
    'Position',[0.0079    0.8201    0.9868    0.1751]);
grid on;
ax=gca;
pos=get(ax,'Position');
set(ax,'Position',[1.1*pos(1) pos(2) pos(3) 0.82*pos(4)]);
% text(28,-1.3,'(a)');
% text(91,-1.3,'(b)');

% %% Create video
% i_env=3;
% path="";
% imgFiles = dir(path) ; 
% N = length(imgFiles)-2 ; 
% writerObj = VideoWriter(path+"\..\"+i_env+".avi");
% writerObj.FrameRate = 5;
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=3:N
%     img = imgFiles(i).name ; 
%     I = imread(path+"\"+img) ;
%     writeVideo(writerObj, I);
% end
% % close the writer object
% close(writerObj);