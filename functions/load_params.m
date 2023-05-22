function [params] = load_params()
%Load parameters in workspace
params.t_sample=1e-3;%sample time simulink
params.n_joints=31; %30 actuators + 1 feeder
params.h=0.01;%height single actuator
params.n_start = 2; %active actuators at start
params.max_iterations = 100; %maximum ik iterations per function call
params.max_delta_q = 10 * pi / 180; % per actuator per ik iteration
params.delta_s = 1/10* params.h; %per iteration
params.max_delta_qF=2*params.delta_s;  % per ik iteration
params.delta_t = 1; %numerical integration time step (inverse kinematics)
params.qmax = pi/180 * 30;
params.qmin = -params.qmax;
params.n_point_corr=2; %every #n_point_corr-th actuator is included within shape fitting 
params.q0_act=0.001; %joint angle if link is active
params.task_prio_gain=1;%for all tasks
params.shape_gain=1;%task_prio_gain*shape_gain for shape fitting
params.task_type_ee=1;%3T=1, eucl_dist=2, 3T2R=3, 3T3R=4
params.phi_xyz_max=30*pi/180;
params.n_ee_path=5;
params.enable_stage2=0;
params.n_start_stage2=7;
end