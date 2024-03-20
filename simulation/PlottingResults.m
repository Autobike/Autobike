function [] = PlottingResults(test_curve,Results,compare_flag)
% This function plots results (trajectory, angles and velocity) 


% save the states for offline kalman
bikedata_simulation_bikestates = array2table([Results.bike_states.Time Results.bike_states.Data]);
filename_simulation_bikestates = 'bikedata_sim_real_states.csv'; % Specify the filename
bikedata_simulation_bikestates.Properties.VariableNames(1:8) = {'Time', 'X', 'Y', 'Psi', 'Roll', 'Rollrate', 'Delta', 'Velocity'};
writetable(bikedata_simulation_bikestates ,filename_simulation_bikestates);

% save measurement data and estimations for offline kalman
bikedata_simulation = array2table([Results.estimated_states.Time Results.estimated_states.Data Results.delta_e1.Data Results.delta_e2.Data ...
Results.delta_psi.Data Results.delta_ref.Data Results.roll_ref.Data Results.e1.Data Results.e2.Data]);
filename_simulation= 'bikedata_sim_est.csv'; % Specify the filename
bikedata_simulation.Properties.VariableNames(1:15) = {'Time', 'X_estimated', 'Y_estimated', 'Psi_estimated', 'Roll_estimated', 'Rollrate_estimated', 'Delta_estimated', 'Velocity_estimated','delta_e1','delta_e2','delta_psi','delta_ref','ref_roll','error1','error2'};
writetable(bikedata_simulation,filename_simulation);

%name of the plot
Tnumber = 'No test case: General simulation run';

% Change it if you want to show stored signals from previous simulation 
Plot_bikesimulation_results(Tnumber, test_curve, Results, compare_flag);
end