% This script is a toplevel script that executes the users desired example case:

addpath('./Exec_Scripts/');
u_choice = input(sprintf('Specify the desired example case to run:\n\t(1) Sinusoidal Timeseries\n\t(2) Hover to Transition Timeseries\n\t(3) Cruise Climbing Turn Timeseries\n\t(4) Ramp demo\n\t(5) Piecewise Bezier Trajectory\nUser Input: '));

switch u_choice
    case 1
        exam_TS_Sinusoidal_traj;
    case 2
        exam_TS_Hover2Cruise_traj
    case 3
        exam_TS_Cruise_Climb_Turn_traj
    case 4
        exam_RAMP
    case 5
        if ~exist("userStruct",'var')
            addpath('./Bez_Functions/');
        end
        exam_Bezier;
    otherwise
        fprintf('User needs to supply selection choice (1-5)\n')
        return
end

% Execute the model
SimPar_STRUCT = SimPar.Value;
sim(model);
% Create sample output plots
%simPlots_GUAM;

%% Plot some stuff

SimOut = logsout{1}.Values;

figure()
subplot(2,1,1)
plot(SimOut.Env.Atmosphere.Density.Time / 60, ...
    SimOut.Env.Atmosphere.Density.Data, LineWidth=2, DisplayName= "Density")
grid minor
xlabel('Time (min)')
ylabel('Air Density (units?)')
legend show

subplot(2,1,2)
plot(SimOut.Env.Atmosphere.SpeedOfSound.Time / 60, ...
    SimOut.Env.Atmosphere.SpeedOfSound.Data, 'LineWidth', 2, ...
    'DisplayName', "Speed of Sound")
grid minor
xlabel('Time (min)')
ylabel('Air Speed of Sound (units?)')
legend show

figure()
plot(SimOut.Vehicle.EOM.WorldRelativeData.AltMSL.Time, ...
    SimOut.Vehicle.EOM.WorldRelativeData.AltMSL.Data, 'LineWidth', 2)
xlabel('Time')
ylabel('Altitude (MSL) (feet?)')