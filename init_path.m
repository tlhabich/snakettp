%Initialize path
base_path = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(base_path, "functions")),...
    '../discrete-frechet-distance/'...
);
     run('../set_based_task_priority_ik/init_path.m');     