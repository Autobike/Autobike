function [] = Plot_MPC_tuning_results(Ts, foldername)
% Trajectory
load("batch_simulations/" + foldername + "/tuning_setup.mat", "sim_tbl")
pred_horiz_span = sim_tbl.pred_horiz_span;
Pf_e1_span = sim_tbl.Pf_e1_span;
Pf_e2_span = sim_tbl.Pf_e2_span;
Q_e1_span = sim_tbl.Q_e1_span;
Q_e2_span = sim_tbl.Q_e2_span;
foldername = "batch_simulations/" + foldername;
simulation_real_states_LQR = readtable(foldername + "/bikedata_simulation_real_statesLQR");
simulation_meas_states_LQR = readtable(foldername + "/bikedata_simulationLQR");
test_curve = readtable(foldername +"/trajectorymat.csv");

% Only trajectory
figure('Name',"Trajectory plot",'Position',[0 0 1920 1080]);
legend_cell = {};
hold on;
plot3(test_curve.Var1,test_curve.Var2,1:length(test_curve.Var1),'o');
legend_cell{end+1} = "Reference";
ax = gca;
% LQR
ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
plot3(simulation_real_states_LQR.X,simulation_real_states_LQR.Y,simulation_real_states_LQR.Time);
ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
plot3(simulation_meas_states_LQR.X_est,simulation_meas_states_LQR.Y_est,simulation_meas_states_LQR.Time,'--');
legend_cell{end+1} = "True: LQR";
legend_cell{end+1} = "Estimated: LQR";
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
            plot3(simulation_real_states.X,simulation_real_states.Y,simulation_real_states.Time);
            ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
            plot3(simulation_meas_states.X_est,simulation_meas_states.Y_est,simulation_meas_states.Time,'--');
            legend_cell{end+1} = "True: ["+pred_horiz_span(i)+","+Pf_e1_span(j)+","+Pf_e2_span(j)+","+Q_e1_span(k)+","+Q_e2_span(k)+"]";
            legend_cell{end+1} = "Estimated: ["+pred_horiz_span(i)+","+Pf_e1_span(j)+","+Pf_e2_span(j)+","+Q_e1_span(k)+","+Q_e2_span(k)+"]";
        end
    end
end
view(0,90)
axis equal
grid on;
legend(legend_cell,'Location','northwest');
xlabel('X-dir [m]');
ylabel('Y-dir [m]');
title('Trajectory');


% Trajectory plot
figure('Name',"Trajectory plots",'Position',[0 0 1920 1080]);
tiledlayout(3,2,'TileSpacing','tight')
nexttile([3,1])
legend_cell = {};
hold on;
plot3(test_curve.Var1,test_curve.Var2,1:length(test_curve.Var1),'o');
legend_cell{end+1} = "Reference";
ax = gca;
% LQR
ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
plot3(simulation_real_states_LQR.X,simulation_real_states_LQR.Y,simulation_real_states_LQR.Time);
ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
plot3(simulation_meas_states_LQR.X_est,simulation_meas_states_LQR.Y_est,simulation_meas_states_LQR.Time,'--');
legend_cell{end+1} = "True: LQR";
legend_cell{end+1} = "Estimated: LQR";
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
            plot3(simulation_real_states.X,simulation_real_states.Y,simulation_real_states.Time);
            ax.ColorOrderIndex = (length(legend_cell)-1)/2+2;
            plot3(simulation_meas_states.X_est,simulation_meas_states.Y_est,simulation_meas_states.Time,'--');
            legend_cell{end+1} = "True: ["+pred_horiz_span(i)+","+Pf_e1_span(j)+","+Pf_e2_span(j)+","+Q_e1_span(k)+","+Q_e2_span(k)+"]";
            legend_cell{end+1} = "Estimated: ["+pred_horiz_span(i)+","+Pf_e1_span(j)+","+Pf_e2_span(j)+","+Q_e1_span(k)+","+Q_e2_span(k)+"]";
        end
    end
end
view(0,90)
axis equal
grid on;
legend(legend_cell,'Location','northwest');
xlabel('X-dir [m]');
ylabel('Y-dir [m]');
title('Trajectory');

% Lateral error
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,simulation_meas_states_LQR.error1);
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, simulation_meas_states.error1);
            m = m + 1;
        end
    end
end
xlabel('Time [s]')
ylabel('Distance [m]')
title('Lateral error')
grid on;

% Heading error
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,rad2deg(simulation_meas_states_LQR.error2));
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, rad2deg(simulation_meas_states.error2));
            m = m + 1;
        end
    end
end
xlabel('Time [s]')
ylabel('Angle [deg]')
title('Heading error')
grid on;

% Steer rate
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,rad2deg(simulation_meas_states_LQR.Steerrate_input));
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, rad2deg(simulation_meas_states.Steerrate_input));
            m = m + 1;
        end
    end
end
xlabel('Time [s]')
ylabel('Angle rate [deg/s]')
title('Steering rate')
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',"State plots",'Position',[0 0 1920 1080]);

% States
tiledlayout(3,2,"TileSpacing","tight")

% X
nexttile
hold on;
ax = gca;
legend_cell = {};
% LQR
ax.ColorOrderIndex = (length(legend_cell))/2+2;
plot(simulation_real_states_LQR.Time,simulation_real_states_LQR.X);
ax.ColorOrderIndex = (length(legend_cell))/2+2;
plot(simulation_meas_states_LQR.Time,simulation_meas_states_LQR.X_est,'--');
legend_cell{end+1} = "True X: LQR";
legend_cell{end+1} = "Estimated X: LQR";
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = length(legend_cell)/2+2;
            plot(simulation_real_states.Time, simulation_real_states.X);
            ax.ColorOrderIndex = length(legend_cell)/2+2;
            plot(simulation_meas_states.Time, simulation_meas_states.X_est,'--');
            legend_cell{end+1} = "True X: ["+pred_horiz_span(i)+","+Pf_e1_span(j)+","+Pf_e2_span(j)+","+Q_e1_span(k)+","+Q_e2_span(k)+"]";
            legend_cell{end+1} = "Estimated X: ["+pred_horiz_span(i)+","+Pf_e1_span(j)+","+Pf_e2_span(j)+","+Q_e1_span(k)+","+Q_e2_span(k)+"]";
        end
    end
end
xlabel('Time [t]');
ylabel('Position X [m]');
legend(legend_cell,'Location','northwest')
grid on;
title('X-coordinate');

% Roll
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_real_states_LQR.Time,rad2deg(simulation_real_states_LQR.Roll));
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,rad2deg(simulation_meas_states_LQR.Roll_est));
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_real_states.Time, rad2deg(simulation_real_states.Roll));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, rad2deg(simulation_meas_states.Roll_est),'--');
            m = m + 1;
        end
    end
end
xlabel('Time [t]');
ylabel('Angle [deg]');
grid on;
title('Roll');

% Y
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_real_states_LQR.Time,simulation_real_states_LQR.Y);
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,simulation_meas_states_LQR.Y_est);
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_real_states.Time, simulation_real_states.Y);
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, simulation_meas_states.Y_est,'--');
            m = m + 1;
        end
    end
end
xlabel('Time [t]');
ylabel('Y [m]');
grid on;
title('Y-coordinate');

% Roll rate
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_real_states_LQR.Time,rad2deg(simulation_real_states_LQR.Rollrate));
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,rad2deg(simulation_meas_states_LQR.Rollrate_est));
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_real_states.Time, rad2deg(simulation_real_states.Rollrate));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, rad2deg(simulation_meas_states.Rollrate_est),'--');
            m = m + 1;
        end
    end
end
xlabel('Time [t]');
ylabel('Angle rate [deg/s]');
grid on;
title('Roll rate');

% Psi
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_real_states_LQR.Time,rad2deg(simulation_real_states_LQR.Psi));
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,rad2deg(simulation_meas_states_LQR.Psi_est));
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_real_states.Time, rad2deg(simulation_real_states.Psi));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, rad2deg(simulation_meas_states.Psi_est),'--');
            m = m + 1;
        end
    end
end
xlabel('Time [t]');
ylabel('Angle [deg]');
grid on;
title('Heading');

% Steer angle
nexttile
hold on;
ax = gca;
m = 2;
% LQR
ax.ColorOrderIndex = m;
plot(simulation_real_states_LQR.Time,rad2deg(simulation_real_states_LQR.Delta));
ax.ColorOrderIndex = m;
plot(simulation_meas_states_LQR.Time,rad2deg(simulation_meas_states_LQR.Delta_est));
m = m + 1;
% MPC
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            simulation_real_states = readtable(foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k));
            simulation_meas_states = readtable(foldername + "/bikedata_simulation"+string(i)+string(j)+string(k));
            ax.ColorOrderIndex = m;
            plot(simulation_real_states.Time, rad2deg(simulation_real_states.Delta));
            ax.ColorOrderIndex = m;
            plot(simulation_meas_states.Time, rad2deg(simulation_meas_states.Delta_est),'--');
            m = m + 1;
        end
    end
end
xlabel('Time [t]');
ylabel('Angle [deg]');
grid on;
title('Steer angle');

end