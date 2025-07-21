
close all
clear
clc

% Add everything in the GUAM repo to the search path:
addpath(genpath("."))

% Load pre-determined trim points from .mat file:
load("vehicles/Lift+Cruise/Trim_poly_XEQ_ConcatV4p0.mat", "XEQ")
trim_points_table = XEQ;
clear XEQ

% Prepare input arguments for the function that we will call to obtain the
% linearized model matrices for THE FIRST TRIM POINT

trim_pt = trim_points_table(:,1,1);

SimIn.numEngines = 9;
lpc = LpC_model_parameters(SimIn);

rho  = 0.00237717; % slugs/ft^3
grav = 32.17405; % ft/sec^2

xeq = trim_pt(1:8); % Pull out the trim condition state variables
ueq = trim_pt(9:end); % Pull out the trim condition effector variables

NS = 4; % Specify number of aero control surfaces
NP = 9; % Specify number of rotor/propeller control effectors

% Set global POLY to true so that the conditional within
% "get_lin_dynamics_heading.m" is TRUE and we use the polynomial model to
% obtain the Jacobian matrices... to me this seems to be egregious coding
% practice. The option for polynomial or S-function model evaluation should
% simply be an argument to the linearization function... Why use a global??
global POLY
POLY = 1;

% Get the full linearized state-space representation
[A, B, C, D, XU0] = get_lin_dynamics_heading(lpc, xeq, ueq, ...
    NS, NP, rho, grav);

% We have successfully obtained the full linearized model for the first
% trim point in the trim table... For the ordering of states and inputs in
% the linearized model, see lines 12-20 (comments) in the function:
% "../vehicles/Lift+Cruise/Control/get_lin_dynamics_heading.m" for a
% description

%% Setup data for my nonlinear sim

% Clear prior data:
clear A B C D XU0 trim_pt xeq ueq

% Choose operating point and design a feedforward-feedback controller:
trim_pt = trim_points_table(:,1,2);

xeq = trim_pt(1:8); % Pull out the trim condition state variables
ueq = trim_pt(9:end); % Pull out the trim condition effector variables

% Get the full linearized state-space representation
[A_hover, B_hover, C_hover, D_hover, XU0_hover] = ...
    get_lin_dynamics_heading(lpc, xeq, ueq, NS, NP, rho, grav);

% Set up parameter values that are necessary to call "run_LPC_aero.m"
dummy = load("adam_scripts/Model+Units_TEST.mat", "SimIn");
Units = dummy.SimIn.Units;
clear dummy
Model = lpc;
rho = 0.00237717; % air density (0 ft @ Standard Atmos.) - slug/ft^3
a = 1116.45; % speed of sound (0 ft @ Standard Atmos.) - ft/s
surf_alloc_mat = [1, -1, 0, 0; ...
                  1,  1, 0, 0; ...
                  0,  0, 1, 0; ...
                  0,  0, 1, 0; ...
                  0,  0, 0, 1;]; % fixed allocation matrix for surface map

dummy = load("adam_scripts/SimPar_STRUCT.mat", "SimPar_STRUCT");
Actuator = dummy.SimPar_STRUCT.Actuator;
Engine = dummy.SimPar_STRUCT.Engine;

J = lpc.I;
m = lpc.mass;


% Set initial conditions for actuator dynamics integrator and state
% derivative integrator:
init_surf_dyn = surf_alloc_mat * ueq(1:4);
init_eng_dyn = ueq(5:end);

% Set initial state vector:
init_euler = XU0_hover(10:12);
init_vbar = XU0_hover(1:3);
init_pqr = XU0_hover(4:6);

init_x_eom = [init_euler; init_vbar; init_pqr];

% Set trim input vector:
u_trim = ueq;

% Set constant perturbed input vector:
% (order: del_f, del_a, del_e, del_r, om1-om8, om9)

delta_u_CONSTANT = [0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 0];

% Set time step for fixed-step simulation:
dt_sim = 0.005; % 200 Hz

%% Setup data for GUAM_copy sim

% Get rotation matrix from body frame to heading frame:\
init_phi = init_euler(1);
init_theta = init_euler(2);
init_psi = init_euler(3);

R_bar = (rotx(rad2deg(init_phi)).' * roty(rad2deg(init_theta)).').';

% Get initial velocity in body frame coordinates:
init_Vel_bEb = R_bar.' * init_vbar;

% Set initial position and angular velocity:
init_Pos_bii = [0; 0; 0];
init_Omeg_BIb = init_pqr;

% Get initial quaternion transformation from inertial to body frame:
init_Q_i2b = QmultSeq(QrotZ(init_psi),QrotY(init_theta),QrotX(init_phi))';

% Assign initial state for integrator in Vehicle EOM -> Equations of 
% Motion -> Simple
init_state_guam = [init_Vel_bEb; init_Omeg_BIb; init_Pos_bii; init_Q_i2b];

% Run script that preps GUAM copy to be run:
exam_TS_Hover2Cruise_traj_COPY

% Set constant input vector for GUAM copy at location:
% Vehicle Generalized Control -> Lift+Cruise Control -> 
input_vec = u_trim + delta_u_CONSTANT;
engine_cmd_CONSTANT = input_vec(5:end);
surf_cmd_CONSTANT = input_vec(1:4);