%  Simulink example
clear all; clc;

%% get available simulink_opts with default options
% simulink_opts = get_acados_simulink_opts;
% 
% % manipulate simulink_opts
% 
% % inputs
% simulink_opts.inputs.lbx = [];
% simulink_opts.inputs.ubx = [];
% simulink_opts.inputs.yref_0 = [];
% simulink_opts.inputs.yref = [];
% simulink_opts.inputs.yref_e = [];
% % simulink_opts.inputs.x_init = 1;
% % simulink_opts.inputs.reset_solver = 1;
% % 
% % 
% % % outputs
% % simulink_opts.outputs.utraj = 1;
% % simulink_opts.outputs.xtraj = 1;
% % simulink_opts.outputs.cost_value = 1;
% % simulink_opts.outputs.KKT_residual = 0;
% % simulink_opts.outputs.KKT_residuals = 1;
% 
% simulink_opts.samplingtime = '-1';
    % 't0' (default) - use time step between shooting node 0 and 1
    % '-1' - inherit sampling time from other parts of simulink model
% Model bounds
model.n_min = -0.12;  % width of the track [m]
model.n_max = 0.12;  % width of the track [m]

% state bounds
model.throttle_min = -1.0;
model.throttle_max = 1.0;

model.delta_min = -0.40;  % minimum steering angle [rad]
model.delta_max = 0.40;  % maximum steering angle [rad]

% input bounds
model.ddelta_min = -2.0;  % minimum change rate of stering angle [rad/s]
model.ddelta_max = 2.0;  % maximum change rate of steering angle [rad/s]
model.dthrottle_min = -10;  % -10.0  % minimum throttle change rate
model.dthrottle_max = 10;  % 10.0  % maximum throttle change rate

% nonlinear constraint
constraint.alat_min = -4;  % maximum lateral force [m/s^2]
constraint.alat_max = 4;  % maximum lateral force [m/s^1]

constraint.along_min = -4;  % maximum lateral force [m/s^2]
constraint.along_max = 4;  % maximum lateral force [m/s^2]

% Define initial conditions
model.x0 = [-2, 0, 0, 0, 0, 0];


%% Run minimal example
%
main;
% minimal_example_ocp;

%% Compile Sfunctions
cd c_generated_code

make_sfun_sim; % integrator
make_sfun; % ocp solver
%%
source_folder = fullfile(pwd, '..');
target_folder = pwd;
copyfile( fullfile(source_folder, 'simulink_implementation.slx'), target_folder );

%% Open Simulink example block
open_system(fullfile(target_folder, 'simulink_implementation'))

%% Run the Simulink model
try
    sim('simulink_implementation.slx');
    cd ..
catch
    cd ..
    error('simulink implementation example failed')
end