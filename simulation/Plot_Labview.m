%% Read and plot Labview data
% Go to FileZilla, save your data.csv file into Labview_data folder
clear
close all
clc

%% Load Data
Labview_data=readtable('Labview_data\data.csv'); % Preferably keep this always the same for simplicity. Input your log data there.
Table_traj = readtable('Traj_ref_test/new_trajectoryfile.csv'); % Input Reference trajectory path and name

% set the initial global coordinate system for gps coordinates
    gps_delay = 5;
% Choose The Bike - Options:'red','black','green','scooter','plastic','MHD' 
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 

%% Prepare data for ploting
% If you're doing outdoor test then set that value to 1
outdoor_test = 1;

% if outdoor_test == 0
%     % Delete all the data that are obtained before reseting the trajectory
%     reset_traj = find(Labview_data.PICurrent_A_~=0,1,'first');
%     Labview_data(1:reset_traj,:) = [];
% end

if outdoor_test == 1
    % Delete all the data that are obtained before reseting the trajectory
    reset_traj = find(Labview_data.ResetTraj==1,1,'last');
    Labview_data(1:reset_traj,:) = [];
end

%obtain X/Y position
longitude0 = deg2rad(11);
latitude0 = deg2rad(57);
Earth_rad = 6371000.0;

X = Earth_rad * (Labview_data.LongGPS_deg_ - longitude0) * cos(latitude0);
Y = Earth_rad * (Labview_data.LatGPS_deg_ - latitude0);

% Obtain the relative time of the data
Labview_data.Time = (Labview_data.Time_ms_- Labview_data.Time_ms_(1))*0.001;

% Obtain the measurements
ay = -Labview_data.AccelerometerY_rad_s_2_;
omega_x = Labview_data.GyroscopeX_rad_s_;
omega_z = Labview_data.GyroscopeZ_rad_s_;
delta_enc = Labview_data.SteeringAngleEncoder_rad_;
v_enc = Labview_data.SpeedVESC_rad_s_*bike_params.r_wheel;
v_ref = Labview_data.SpeedReference_rad_s_*bike_params.r_wheel;
v_GPS=Labview_data.GPSVelocity_m_s_;

% Prepare measurement data for the offline kalman
gps_init = find(Labview_data.GNSSFlag > gps_delay, 1 );
measurementsGPS = [Labview_data.Time X Y];
measurementsGPS(1:gps_init,:) = [];
X(1:gps_init) = [];
Y(1:gps_init) = [];
measurements = [Labview_data.Time ay omega_x omega_z delta_enc v_enc];
measurements(1,:) = [];
steer_rate = [Labview_data.Time Labview_data.SteerrateInput_rad_s_];
steer_rate(1,:) = [];
gpsflag = [Labview_data.Time Labview_data.GNSSFlag];

% Decide length of the plot
start_point = 1;
end_point = length(measurementsGPS)-1;


% Trajectory
fig = figure();
plot3(Labview_data.StateEstimateX_m_(start_point:end_point) - X(1) ,Labview_data.StateEstimateY_m_(start_point:end_point) - Y(1),Labview_data.Time(start_point:end_point))
hold on
plot3(measurementsGPS(start_point:end_point,2) - X(1),measurementsGPS(start_point:end_point,3) - Y(1),Labview_data.Time(start_point:end_point),'*')
plot3(Table_traj.Var1(:)-Table_traj.Var1(1),Table_traj.Var2(:)-Table_traj.Var2(1),1:length(Table_traj.Var1(:))) %,'Trajectory reference'
% plot3(sim_data.X_estimated(:,1), % sim_data.Y_estimated(:,1),sim_data.Time(:,1)) %'Estimated sim'
view(0,90)
xlabel('X position (m)')
ylabel('Y position (m)')
axis equal
legend('Estimated test','Measurements');
title('Trajectory')

% States
fig = figure();
subplot(4,2,1)
plot(Labview_data.Time(start_point:end_point), Labview_data.StateEstimateX_m_(start_point:end_point)-measurementsGPS(1,2))
hold on
% plot(sim_data.Time(:,1),sim_data.X_estimated(:,1)) %'Simulation',
plot(measurementsGPS(start_point:end_point,1),measurementsGPS(start_point:end_point,2)-measurementsGPS(1,2))
xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('Online estimation','GPS measurements')

subplot(4,2,3)
plot(Labview_data.Time(start_point:end_point), Labview_data.StateEstimateY_m_(start_point:end_point)-measurementsGPS(1,3))
hold on
plot(measurementsGPS(start_point:end_point,1),measurementsGPS(start_point:end_point,3)-measurementsGPS(1,3))
% plot(sim_data.Time(:,1),sim_data.X_estimated(:,1)) %'Simulation',
xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('Online estimation','GPS measurements')

subplot(4,2,5)
plot(Labview_data.Time(start_point:end_point), rad2deg(wrapToPi(Labview_data.StateEstimatePsi_rad_(start_point:end_point))))
% hold on
% plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
% plot(sim_data.Time(:,1), rad2deg(wrapToPi(sim_data.Psi_estimated(:,1)))) %'Simulation',
xlabel('Time (s)')
ylabel('heading (deg)')
grid on
legend('Online estimation') %,'Steering motor enabled flag')

subplot(4,2,2)
plot(Labview_data.Time(start_point:end_point),rad2deg(Labview_data.StateEstimateRoll_rad_(start_point:end_point)))
hold on
plot(Labview_data.Time(start_point:end_point),rad2deg(Labview_data.Input(start_point:end_point)))
plot(Labview_data.Time(start_point:end_point), 2*Labview_data.steeringFlag(start_point:end_point),'--','LineWidth',2)
% plot(sim_data.Time(:,1), rad2deg(sim_data.Roll_estimated(:,1))) %'Simulation',
% plot(sim_data.Time(:,1),rad2deg(sim_data.ref_roll(:,1))) %'rollref sim'
xlabel('Time (s)')
ylabel('Roll (deg)')
grid on
legend('Online estimation','rollref used','Steering motor enabled flag')

subplot(4,2,4)
plot(Labview_data.Time(start_point:end_point), rad2deg(Labview_data.StateEstimateRollrate_rad_s_(start_point:end_point)))
hold on
plot(Labview_data.Time(start_point:end_point), rad2deg(Labview_data.GyroscopeX_rad_s_(start_point:end_point)))
plot(Labview_data.Time(start_point:end_point), 10 * Labview_data.steeringFlag(start_point:end_point),'--','LineWidth',2)
% plot(sim_data.Time(:,1), rad2deg(sim_data.Rollrate_estimated(:,1))) %'simulation'
xlabel('Time (s)')
ylabel('Roll Rate (deg/s)')
grid on
legend('Online estimation', 'Measurement','Steering motor enabled flag')

subplot(4,2,6)
plot(Labview_data.Time(start_point:end_point),rad2deg(Labview_data.StateEstimateDelta_rad_(start_point:end_point)))
hold on
plot(Labview_data.Time(start_point:end_point),rad2deg(Labview_data.SteeringAngleEncoder_rad_(start_point:end_point)))
plot(Labview_data.Time(start_point:end_point), 5*Labview_data.steeringFlag(start_point:end_point),'--','LineWidth',2)
% plot(sim_data.Time(:,1), rad2deg(sim_data.Delta_estimated(:,1))) %'simulation', 
xlabel('Time (s)')
ylabel('Steering Angle (deg)')
grid on
legend('Online estimation', 'Measurement','Steering motor enabled flag')

subplot(4,2,8)
plot(Labview_data.Time(start_point:end_point), Labview_data.StateEstimateVelocity_m_s_(start_point:end_point))
hold on
plot(Labview_data.Time(start_point:end_point), v_enc(start_point:end_point))
plot(Labview_data.Time(start_point:end_point), v_GPS(start_point:end_point))
plot(Labview_data.Time(start_point:end_point), v_ref(start_point:end_point))
% plot(sim_data.Time(:,1), sim_data.Velocity_estimated(:,1)) %'simulation',
xlabel('Time (s)')
ylabel('velocity (m/s)')
ylim([-1 5])
grid on
legend('Online estimation','Vesc Measurement','GPS Measurement','Speed Reference')

% Steering motor rate input
figure
plot(Labview_data.Time(start_point:end_point), Labview_data.SteerrateInput_rad_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('Steering rate (rad/s)')
grid on
legend('Steering Motor Input')

% Forward motor current
figure
plot(Labview_data.Time(start_point:end_point), Labview_data.PICurrent_A_(start_point:end_point))
hold on
plot(Labview_data.Time(start_point:end_point), Labview_data.InputCurrent_A_(start_point:end_point))
plot(Labview_data.Time(start_point:end_point), Labview_data.MotorCurrent_A_(start_point:end_point))
xlabel('Time (s)')
ylabel('Forward Motor Current (A)')
grid on
legend('PI current(A)','Input Current(A)','Motor Current(A)')

% Accelerometer
figure
plot(Labview_data.Time(start_point:end_point), Labview_data.AccelerometerY_rad_s_2_(start_point:end_point))
xlabel('Time (s)')
ylabel('Accelerometer Y-axis (m/s^2)')
grid on
legend('Accelerometer Y')

if outdoor_test == 1

    % Delta contributions
    % test = rad2deg(-0.008944 .* sign(Labview_data.Error1(:,1)) .* min(abs(Labview_data.Error1(:,1)),5.66548));
    figure()
    subplot(3,1,1)
    plot(Labview_data.Time(start_point:end_point,1),rad2deg(Labview_data.LateralContribution(start_point:end_point,1)));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),rad2deg(sim_data.delta_e1(:,1))); %'simulation',
    % plot(Labview_data.Time(start_point:end_point,1),test(start_point:end_point,1)); %'test',
    xlabel('Time [t]')
    ylabel('Angle [Deg]')
    legend('Onlime estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('lateral error contribution')
    
    subplot(3,1,2)
    plot(Labview_data.Time(start_point:end_point,1),rad2deg(Labview_data.HeadingContribution(start_point:end_point,1)));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),rad2deg(sim_data.delta_e2(:,1))); %'simulation',
    xlabel('Time [t]')
    ylabel('Angle [Deg]')
    legend('Onlime estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('Heading error contribution')
    
    subplot(3,1,3)
    plot(Labview_data.Time(start_point:end_point,1),rad2deg(Labview_data.DpsirefContribution(start_point:end_point,1)));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),rad2deg(sim_data.delta_psi(:,1))); %'simulation',
    xlabel('Time [t]')
    ylabel('Angle [Deg]')
    legend('Online estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('Dpsiref contribution')
    
    % Compare delta_ref and roll_ref
    sum_cont = rad2deg(Labview_data.DpsirefContribution)+rad2deg(Labview_data.HeadingContribution)+rad2deg(Labview_data.LateralContribution);
    figure();
    subplot(2,1,1)
    plot(Labview_data.Time(start_point:end_point,1),sum_cont(start_point:end_point,1));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),rad2deg(sim_data.delta_ref(:,1))); %'simulation',
    xlabel('Time [t]')
    ylabel('Angle [Deg]')
    legend('online estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('Delta_{ref}')
    
    subplot(2,1,2)
    plot(Labview_data.Time(start_point:end_point,1),rad2deg(Labview_data.Rollref(start_point:end_point,1)));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),rad2deg(sim_data.ref_roll(:,1))); %'simulation',
    xlabel('Time [t]')
    ylabel('Angle [Deg]')
    legend('online estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('Roll_{ref}')
    
    % Lateral and heading error
    figure();
    subplot(2,1,1)
    plot(Labview_data.Time(start_point:end_point,1),Labview_data.Error1(start_point:end_point,1));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),sim_data.error1(:,1)); %'simulation',
    xlabel('Time [t]')
    ylabel('Distance [m]')
    legend('online estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('Lateral error')
    
    subplot(2,1,2)
    plot(Labview_data.Time(start_point:end_point,1),rad2deg(Labview_data.Error2(start_point:end_point,1)));
    hold on
    plot(Labview_data.Time(start_point:end_point), Labview_data.steeringFlag(start_point:end_point))
    % plot(sim_data.Time(:,1),rad2deg(sim_data.error2(:,1))); %'simulation',
    xlabel('Time [t]')
    ylabel('Angle [Deg]')
    legend('online estimation','Steering motor enabled flag','Location','southeast')
    grid on
    title('Heading error')

end
