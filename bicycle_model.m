function [model, constraint] = bicycle_model(track)

import casadi.*
model = struct();
constraint = struct();

model_name = 'Spatialbicycle_model';

% load track parameters
[s0, ~, ~, ~, kapparef] = getTrack(track);
len = length(s0);
pathlength = s0(end);
% copy loop to beginning and end
s0 = [s0; s0(end) + s0(2:end)];
kapparef = [kapparef; kapparef(2:end)];
s0 = [-s0(end-1) + s0(end-80 : end-2); s0];
kapparef = [kapparef(end-79 : end-1); kapparef];

% compute spline interpolations
kapparef_s = interpolant('kapparef_s', 'bspline', {s0}, kapparef);

%% Race car parameters
% m = 0.043;
m = MX.sym('m');
C1 = 0.5;
C2 = 15.5;
Cm1 = 0.28;
Cm2 = 0.05;
Cr0 = 0.011;
Cr2 = 0.006;

%% CasADi Model
% set up states & controls
s = MX.sym('s');
n = MX.sym('n');
alpha = MX.sym('alpha');
v = MX.sym('v');
D = MX.sym('D');
delta = MX.sym('delta');
x = vertcat(s, n, alpha, v, D, delta);

% controls
derD = MX.sym('derD');
derDelta = MX.sym('derDelta');
u = vertcat(derD, derDelta);

% xdot
sdot = MX.svym('sdot');
ndot = MX.sym('ndot');
alphadot = MX.sym('alphadot');
vdot = MX.sym('vdot');
Ddot = MX.sym('Ddot');
deltadot = MX.sym('deltadot');
xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot);

% algebraic variables
z = [];

% parameters
p = [];

% dynamics
Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v);
sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n);
f_expl = vertcat(...
    sdota,...
    v * sin(alpha + C1 * delta),...
    v * C2 * delta - kapparef_s(s) * sdota,...
    Fxd / m * cos(C1 * delta),...
    derD,...
    derDelta...
);

% constraint on forces
a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m;
a_long = Fxd / m;

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

% define constraints struct
constraint.alat = Function('a_lat', {x, u}, {a_lat});
constraint.pathlength = pathlength;
constraint.expr = vertcat(a_long, a_lat, n, D, delta);
sym_p = m;

% Define model struct
params = struct();
params.C1 = C1;
params.C2 = C2;
params.Cm1 = Cm1;
params.Cm2 = Cm2;
params.Cr0 = Cr0;
params.Cr2 = Cr2;

model.f_impl_expr = f_expl - xdot;
model.f_expl_expr = f_expl;
model.x = x;
model.xdot = xdot;
model.u = u;
model.z = z;
model.sym_p = sym_p;
model.name = model_name;
model.params = params;

end
