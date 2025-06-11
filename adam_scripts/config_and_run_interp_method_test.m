
close all
clear
clc

% Load XU0_interp array:
load("../vehicles/Lift+Cruise/trim_table_Poly_ConcatVer4p0.mat", ...
    "XU0_interp")

% Load frozen SimIn structure:
load("SimIn_FROZEN.mat")

ubar_cmd = 0.5 * (XU0_interp(1,4,2) + XU0_interp(1,3,2));
wbar_cmd = XU0_interp(3,3,2);

out = sim("test_interp_method.slx");

test_pt = (XU0_interp(:,4,2) + XU0_interp(:,3,2))/2;
interp_pt = [out.x0_interpolated(:,:,1); out.u0_interpolated(:,:,1)];

equality_check = (test_pt == interp_pt);

compare = [test_pt, interp_pt];