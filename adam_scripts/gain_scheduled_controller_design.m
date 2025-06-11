
close all
clear
clc

% Add everything in the GUAM repo to the search path:
addpath(genpath(".."))

% The purpose of this script is to obtain the linearized models for each of
% the trim points identified in the Lift+Cruise model in GUAM, and then
% design the baseline feedforward-feedback controllers that we will use for
% the gain scheduling study.

% Load pre-determined trim points from .mat file:
load("../vehicles/Lift+Cruise/Trim_poly_XEQ_ConcatV4p0.mat", "XEQ")
trim_points_table = XEQ;
clear XEQ

% Prepare input arguments for the function that we will call to obtain the
% linearized model matrices for THE FIRST TRIM POINT (will create a loop
% after a successful test with only one trim point)

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
% trim point in the trim table... Next step is to set up a loop that
% obtains all of the ABCD matrices and XU0 points for all (84) trim
% points. I think it will be good to first think a little bit about how we
% are going to obtain the feedback gains and how we are going to
% interpolate between controller gains before doing this loop. For the
% ordering of states and inputs in the linearized model, see lines
% 12-20 (comments) in the function:
% "../vehicles/Lift+Cruise/Control/get_lin_dynamics_heading.m" for a
% description

%% Test Nonlinear Sim

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
dummy = load("Model+Units_TEST.mat", "SimIn");
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

dummy = load("SimPar_STRUCT.mat", "SimPar_STRUCT");
Actuator = dummy.SimPar_STRUCT.Actuator;
Engine = dummy.SimPar_STRUCT.Engine;

J = lpc.I;
m = lpc.mass;


% Set initial conditions for actuator dynamics integrator and state
% derivative integrator:
init_surf_dyn = surf_alloc_mat * ueq(1:4);
init_eng_dyn = ueq(5:end);

init_euler = XU0_hover(10:12);
init_vbar = XU0_hover(1:3);
init_pqr = XU0_hover(4:6);

init_x_eom = [init_euler; init_vbar; init_pqr];

% Set time step for fixed-step simulation:
dt_sim = 0.005; % 200 Hz

%% Design FF+FB controller for hover trim point:

% Set trim states and inputs for computing perturbed states/inputs for
% linear controller:
euler_trim = XU0_hover(10:12);
vbar_trim = XU0_hover(1:3);
pqr_trim = XU0_hover(4:6);
x_trim = [euler_trim; vbar_trim; pqr_trim];
u_trim = XU0_hover(13:end);

% Find linearization SS matrices for reduced model:
rA_hover = A_hover(4:end, 4:end);
rB_hover = B_hover(4:end, :);
rC_hover = eye(9);
rD_hover = zeros(9, 13);

% Set cost matrices:
Q = diag([100, 100, 100, 100, 100, 100, 100, 100, 100]);
R = diag([10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 100]);

% Obtain feedback gain:
[K_hover, S, P] = lqr(rA_hover, rB_hover, Q, R, []);