
close all
clear
clc

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

rho  = 0.0023769; % slugs/ft^3
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