function [out, ctrl_error] = ctrl_lon(aircraft, xu_eq, rho, grav, q, r, wc, FreeVar_pnt, Trans_pnt)
% *************************************************************************
% function [ out ctrl_error] = ctrl_lon(aircraft, xu_eq, rho, grav, q,r,wc)
%
% This script takes input of an aero/propulsive model (strip theory or
% polynomial), a particular trim condition (one point of a trim schedule),
% and desired optimization weights and outputs a unified longitudinal
% controller.
%
% REFERENCES: 
% Ref 1 (unified controller): "Examination of Unified Control Approaches Incorporating
% Generalized Control Allocation", AIAA 2021-0999
%
% Ref 2 (strip theory aero/propulsive model): "A Strip Theory Approach to
% Dynamic Modeling of Tiltwing eVTOL Aircraft", AIAA 2021-1720
% 
% Ref 3 (polynomial aero/propulsive model): "Full-Envelope Aero-Propulsive Model
% Identification for Lift+Cruise Aircraft Using Computational Experiments",
% AIAA 2021-3170
%
% Written by Jacob Cook, NASA Langley Research Center.
% Current contact michael.j.acheson@nasa.gov, (757)-864-9457
% Dynamics Systems and Control Branch (DSCB D-316)
%
% INPUTS:
%   aircraft: Either matlab aircraft class object (strip theory), or
%       structure of L+C data (polynomial)
%   xu_eq: Vector of trim states and controls, 
%       Order: (ubar, wbar, turn radius, theta, phi, p (body frame roll rate),
%        q, r, flap, ail, elev, rud, lifting rotors (1-8), pusher prop)
%   rho: scalar value of air density (slugs/ft^3)
%   grav: scalar value of gravity acceleration (ft/sec^2)
%   q: Desired state cost weights
%   r: Desired control acceleration cost weights
%   wc: Desired control allocation weights
%   FreeVar_pnt: Arrays of variables (including effectors) avail for control use
%   Trans_pnt: Column vector of transition start velocity and transition end velocity
% OUTPUTS:
%   out: Structure of linerized unified controller gains etc.. (see
%       get_long_dynamics_heading.m and Ref 1 for details)
%   ctrl_error: Scalar flag indicating error in controller design at
%       current trim point
% OTHER UTILIZED FUNCTIONS:
%   get_long_dynamics_heading.m: % Get the linearized longitudinal dynamics of L+C vehicle 
% *************************************************************************

% VERSION HISTORY
% 7.16.2021, Jacob Cook (NASA LaRC D-316): Initial version for use with
% NASA Lift+Cruise (L+C) vehicle in GVS
% 
% 7.20.2023, Michael J. Acheson (NASA LaRC D-316): Updated version to
% include documentation, changes to output B matrices (based on
% desired active control effectors, and inclusion of flaps capability as an
% active control effector
% 

ctrl_error = 0; % Initialize controller design error flag to false

Q = diag(q);
R = diag(r);
Wc = diag(wc);

% State space dynamics in the control frame
xeq = xu_eq(1:8); % Pull out the trim condition state variables

ueq = [xu_eq(9:end)]; % Pull out the trim condition effector variables

NS = 4; % Specify number of aero control surfaces
NP = 9; % Specify number of rotor/propeller control effectors

% Obtain the full longitudinal (linearized) state-space matrices and
% full trim vector
[Alon, Blon, Clon, Dlon, XU0, A_full, B_full, C_full, D_full] = get_long_dynamics_heading(aircraft, xeq, ueq, NS, NP, rho, grav, FreeVar_pnt, Trans_pnt);

if xeq(1) > Trans_pnt(2) % Zero out the lifting rotors after the trans regime ends
    Blon(:,1:8) = zeros(4,8);
end

% size definitions
Nx  = 4;  % system states
Ni  = 3;  % integrator states
Nr  = 3;  % reference 
Nu  = 11; % physical controls
Nv  = 1;  % virtual controls
Nmu = 3;  % general inputs
Nxi = 3;  % general states

% Performance design with general acceleration inputs
Av = Alon([1 2 3], [1 2 3]);
Bv = eye(Nxi);
Cv = eye(Nxi);
Dv = zeros(Nxi);

At = [ zeros(Ni,Ni)   Cv   ;
       zeros(Nxi,Ni)   Av  ];
Bt = [ Dv; Bv];

% LQR optimal feedback gains
[Kc, P, CLP] = lqr(At,Bt,Q,R);
Ki0 = Kc(:,1:Ni);
Kx0 = Kc(:,Ni+1:Ni+Nxi);

% Control Allocation design 
% Bu = [u u_v]
Bu = [ Blon([1 2 3],:) Alon([1 2 3], 4) ];

% Define the mapping from general input to 
% physical and virtual control effectors
M = Wc\Bu'/(Bu/(Wc)*Bu'); % M = Wc\Bu'*inv(Bu*inv(Wc)*Bu');

A = Alon;
B = Blon;

% Tracking states
C = Clon([1 2 3],:);
D = Dlon([1 2 3],:);

% add a column of zeros into the 
% state feedback matrix so we can add
% theta back in as a state
Kx = Kx0*[1 0 0 0; 0 1 0 0; 0 0 1 0];
Ki = Ki0;

% Break up the Allocation Matrix
Mu = M(1:Nu,:);
Mv = M(Nu+Nv,:);

% Virtual control 
Cv = Clon(4,:);
Kv = [0; 0; 1];

F = eye(3);
G = zeros(3);

Acl = [ Kv*Mv*Ki    Kv*Mv*Kx+Kv*Cv+C  ;
        -B*Mu*Ki           A-B*Mu*Kx ];

% Check if any eigenvalues of closed loop Acl are positive
if sum(eig(Acl) > 0)
  fprintf('trim point has unstable poles\n')
  ctrl_error = 1;
end

% Assign outputs
out.Ap = A;
out.Bp = B;
out.Cp = eye(Nx);
out.Dp = zeros(Nx,Nu);

out.Ac = Kv*Mv*Ki;
out.Bc = Kv*Mv*Kx+Kv*Cv+C;
out.Br = -eye(Ni,Nr);

out.Cc = -Mu*Ki;
out.Dc = -Mu*Kx;
out.Dr =  zeros(Nu,Nr);

out.Ki = Ki;
out.Kx = Kx;
out.Kv = Kv;

out.F  = F;
out.G  = G;
out.C  = C;
out.Cv = Cv;

out.Q = Q;
out.R = R;

out.W  = Wc;
out.B  = Bu;

% Output the full linearized dynamics state-space matrices
out.A_full = A_full;
out.B_full = B_full;
out.C_full = C_full;
out.D_full = D_full;

out.XU0 = XU0;

