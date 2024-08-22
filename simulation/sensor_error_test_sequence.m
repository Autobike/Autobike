% This is a test sequence that detects if the steering angle encoder, motor velocity measurements or the GPS measurements are faulty. Using measurement data from a predetermined path.

%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

%% Settings and Parameters

% Path to data file and path to trajectory file

%data_path = 'Logging_data\Test_session_02_06\data9.csv';
%traj_path = 'Traj_ref_test\trajectorymat_parking_turn_right_10.csv';

%data_path = 'Logging_data\Test_session_27_06\data_16.csv';
%traj_path = 'Traj_ref_test\trajectorymat_asta0_infinite_35.csv';

data_path = 'Logging_data\Test_session_02_06\data8.csv';
traj_path = 'Traj_ref_test\trajectorymat_parking_turn_left_10.csv';

%data_path = 'Logging_data\Test_session_22_06\data_5.csv';
%traj_path = 'Traj_ref_test\trajectorymat_parking_line.csv';


% Reference velocity of bike
velocity = 3;
% Using simulation data?
simulated_data = false;

% Error detection parameters

% Error detecting threshold
vel_err_treshold = 0.5; % m/s
gps_err_threshold = 0.5; % m
steer_err_threshold = 0.5; % rad

% What percent of data should be outside
% of threshold for error to be flagged?
vel_err_percentage = 5;
gps_err_percentage = 10;
steer_err_percentage = 8;

% Load data
data_lab = readtable(data_path);

% Load trajectory
Table_traj = readtable(traj_path);
Table_traj(1,:) = [];

% General Parameters

    % Gravitational Acceleration
        g = 9.81;
    % Name of the model
        model = 'Parallel_Kalman_offline_sim';
    % Sampling Time
        Ts = 0.01; 
    % Open the Simulink Model
        open([model '.slx']);
    % Choose the solver
        set_param(model,'AlgebraicLoopSolver','TrustRegion');

    
%%%%%%%%%%%%%GUI%%%%%%%%%%%%%%%%%%%%%%%%%
% % reduce/increase simulation time for desired timescale on x-axis of the plots
%     sim_time = 50;
% Take into account a valid speed. 
    v= velocity; 
% set the initial global coordinate system for gps coordinates
    gps_delay = 5;
% Choose The Bike - Options: 'red' or 'black' 
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% selector =1 to cut the data from the moment that steering is on until the end,
% selector=0 to cut the data from a specific point
% selector=2 all the data will be plotted
selector=1;

%% Unpacked bike_params

h = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
c = bike_params.c_mod;
m = bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;
gpsflag = [0 0];
gpsflag_no_gps = [1 1];

% Correction IMU
IMU_x = bike_params.IMU_x_mod;
IMU_roll = bike_params.IMU_roll_mod;
IMU_pitch = bike_params.IMU_pitch_mod;            
IMU_yaw = bike_params.IMU_yaw_mod;

% Convert orientation offsets to radians
roll = IMU_roll * pi / 180;
pitch = IMU_pitch * pi / 180;
yaw = IMU_yaw * pi / 180;

% Calculate transformation matrix
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];   % Rotation matrix for roll
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)]; % Rotation matrix for pitch
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];        % Rotation matrix for yaw

T = Rz*Ry*Rx; 

if simulated_data == false
    %Converting GPS data to X and Y position
    %Setting X and Y to zero at first long/lat datapoint
    [data_lab_,data_lab_idx,data_lab_ic1] = unique(data_lab.StateEstimateX_m_,'stable');
    data_lab = data_lab(data_lab_idx,:);
    % Delete the data before reseting the trajectory and obtain X/Y position
    reset_traj = find(data_lab.ResetTraj==1,1,'last');
    data_lab(1:reset_traj,:) = [];
    longitude0 = deg2rad(11);
    latitude0 = deg2rad(57);
    Earth_rad = 6371000.0;
else
    longitude0 = 0;
    latitude0 = 0;
    Earth_rad = 0;
end

if simulated_data == 0
    X = Earth_rad * (data_lab.LongGPS_deg_ - longitude0) * cos(latitude0);
    Y = Earth_rad * (data_lab.LatGPS_deg_ - latitude0);
else
    X = data_lab.LongGPS_deg_;
    Y = data_lab.LatGPS_deg_;
end



% Obtain the relative time of the data
data_lab.Time = (data_lab.Time_ms_- data_lab.Time_ms_(1))*0.001;

% Obtain the measurements
ay = data_lab.AccelerometerY_rad_s_2_;
omega_x = data_lab.GyroscopeX_rad_s_;
omega_z = data_lab.GyroscopeZ_rad_s_;
delta_enc = data_lab.SteeringAngleEncoder_rad_;
v_enc = data_lab.SpeedVESC_rad_s_*bike_params.r_wheel;
v_GPS=data_lab.GPSVelocity_m_s_;

% Prepare measurement data for the offline kalman
if simulated_data == 0
    gps_init = find(data_lab.flag > gps_delay, 1 );
else
    gps_init = 1;
end
measurementsGPS = [data_lab.Time X Y];
measurementsGPS(1:gps_init,:) = [];
X(1:gps_init) = [];
Y(1:gps_init) = [];
measurements = [data_lab.Time ay omega_x omega_z delta_enc v_enc];
measurements(1,:) = [];
steer_rate = [data_lab.Time data_lab.SteerrateInput_rad_s_];
steer_rate(1,:) = [];
if simulated_data == 0
    gpsflag = [data_lab.Time data_lab.flag];
end

% Translate the trajectory to the point where is reseted
GPS_offset_X = X(1) - Table_traj.Var1(1);
GPS_offset_Y = Y(1) - Table_traj.Var2(1);
Table_traj.Var1(:) = Table_traj.Var1(:) + GPS_offset_X;
Table_traj.Var2(:) = Table_traj.Var2(:) + GPS_offset_Y;

% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,theta)
        initial_state.x = Table_traj.Var1(1);
        initial_state.y = Table_traj.Var2(1);
        initial_state.heading = Table_traj.Var3(2);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;

 %% mirror sample times
% %this part is used to check logging sample time
% %logging
% sampletime_diff_log = diff(data_lab.Time);
% 
% %state estimator
% sampletime_it_filter = diff(data_lab.StateEstimatorIterations);
% sampletime_diff_filter = sampletime_diff_log./sampletime_it_filter;
% 
% %trajectory
% sampletime_it_traj = diff(data_lab.TrajectoryIterations);
% sampletime_diff_traj = sampletime_diff_log./sampletime_it_traj;
% 
% %GPS
% if simulated_data == false
%     % sampletime_logGPS = NaN(length(data_lab.Time)-1,1);
%     Flag_new=unique(data_lab.flag,'row','stable');
%     
%     for i = 1:size(Flag_new)
%          time_index(i)=find((data_lab.flag(:)==Flag_new(i)),1,'first');
%          sampletime_logGPS(i)=data_lab.Time(time_index(i))';
%     end
%     
%     sampletime_it_GPS_short = diff(Flag_new);
%     sampletime_diff_logGPS=diff(sampletime_logGPS);
%     sampletime_diff_GPS = sampletime_diff_logGPS'./sampletime_it_GPS_short;
%     
%     sampletime_diff_GPS_N=NaN(length(data_lab.Time)-1,1);
%     for i = 1:(size(Flag_new)-1)
%          time_index(i)=find((data_lab.flag(:)==Flag_new(i)),1,'first');
%         sampletime_diff_GPS_N(time_index(i))=sampletime_diff_GPS(i);
%     end
% end

%% Kalman Filter
% A matrix (linear bicycle model with constant velocity)
% Est_States := [X Y psi phi phi_dot delta v]
A = [0 0 0 0 0 0 1;
     0 0 v 0 0 v*(lr/(lf+lr))*sin(lambda) 0;
     0 0 0 0 0 (v/(lr+lf))*sin(lambda) 0;
     0 0 0 0 1 0 0;
     0 0 0 (g/h) 0 ((v^2/h)-(g*lr*c/(h^2*(lr+lf))))*sin(lambda) 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0];

% B matrix (linear bicycle model with constant velocity)
B = [0 0 0 0 ((v*lr)/(h*(lr+lf))) 1 0]';

% Including GPS
C1 = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 -g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 (v)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];

D1 = [0 0 (-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Excluding GPS
C2 = [(-g+((-h_imu*g)/h)) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2)+(v^2)*sin(lambda)/(lr+lf) 0;
      0 1 0 0;
      0 0 (v)*sin(lambda)/(lr+lf) 0;
      0 0 1 0;
      0 0 0 1];

D2 = [(-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Discretize the model
A_d = (eye(size(A))+Ts*A);
B_d = Ts*B;

%% Comparing different Q R

% Load Q and R matrices
load('Q_and_R_backup_red_bike.mat');
% R =Rscale* [R_GPS 0 0 0 0 0 0;
%               0 R_GPS 0 0 0 0 0;
%               0 0 R_ay 0 0 0 0;
%               0 0 0 R_wx 0 0 0;
%               0 0 0 0 R_wz 0 0;
%               0 0 0 0 0 R_delta 0;
%               0 0 0 0 0 0 R_v];
R_gps = R+diag([1000000000 100000000 100000000 0 0 0 0]);
R_vel = R+diag([0 0 0 0 0 0 1000000]);
R_steer = R+diag([0 0 0 0 0 1000000 0]);

% Compute Kalman Gain
% including GPS
    [P1,Kalman_gain1,eig_K] = idare(A_d',C1',Q,R,[],[]);
    eig1_K = abs(eig_K);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag 

    [~,Steer_kalman_gain1,~] = idare(A_d',C1',Q,R_steer,[],[]);
    %eig1_K = abs(eig_K);
    Steer_kalman_gain1 = Steer_kalman_gain1';

    [~,Vel_kalman_gain1,~] = idare(A_d',C1',Q,R_vel,[],[]);
    %eig1_K = abs(eig_K);
    Vel_kalman_gain1 = Vel_kalman_gain1';

    [~,GPS_kalman_gain1,~] = idare(A_d',C1',Q,R_gps,[],[]);
    %eig1_K = abs(eig_K);
    GPS_kalman_gain1 = GPS_kalman_gain1';
  

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
        if abs(Steer_kalman_gain1(i,j)) < 10^-5
            Steer_kalman_gain1(i,j) = 0;
        end
        if abs(Vel_kalman_gain1(i,j)) < 10^-5
            Vel_kalman_gain1(i,j) = 0;
        end
        if abs(GPS_kalman_gain1(i,j)) < 10^-5
            GPS_kalman_gain1(i,j) = 0;
        end
    end
end 

% Kalman_gain excluding GPS
Kalman_gain2 = Kalman_gain1(4:7,3:7);
Steer_kalman_gain2 = Steer_kalman_gain1(4:7,3:7);
Vel_kalman_gain2 = Vel_kalman_gain1(4:7,3:7);
GPS_kalman_gain2 = GPS_kalman_gain1(4:7,3:7);

%save matrices
matrixmat = [A_d; B_d'; C1; D1';Kalman_gain1];

filename_matrix = 'matrixmat.csv'; % Specify the filename
csvwrite(filename_matrix, matrixmat); % Write the matrix to the CSV file

%% Balancing Controller

% Outer loop -- Roll Tracking
P_balancing_outer = 3.75;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3.5;
I_balancing_inner = 0;
D_balancing_inner = 0;  

%% Offline Simulation
sim_time = data_lab.Time(end);

tic
try Results2 = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc
standard_kalman = Results2.sim_Kalman;
steer_kalman = Results2.sim_Steer;
vel_kalman = Results2.sim_Velocity;
gps_kalman = Results2.sim_GPS;


vel_discrepancy = standard_kalman.data(:,7)-vel_kalman.data(:,7);
x_coord_discrepancy = standard_kalman.data(:,1)-gps_kalman.data(:,1);
y_coord_discrepancy = standard_kalman.data(:,2)-gps_kalman.data(:,2);
steer_discrepancy = standard_kalman.data(:,6)-steer_kalman.data(:,6);

vel_dis_index = abs(vel_discrepancy) > vel_err_treshold;
x_dis_index = abs(x_coord_discrepancy) > gps_err_threshold;
y_dis_index = abs(y_coord_discrepancy) > gps_err_threshold;
steer_dis_index = abs(steer_discrepancy) > steer_err_threshold;
N = length(measurements(:,1));
vel_issue = false;
gpsX_issue = false;
gpsY_issue = false;
steer_issue = false;

if sum(vel_dis_index) > round((vel_err_percentage/100)*N)
    vel_issue = true;
    disp('!!! Potential discrepancy in velocity measurement detected !!!')
end
if sum(x_dis_index) > round((gps_err_percentage/100)*N)
    gpsX_issue = true;
    disp('!!! Potential discrepancy in location measurement detected !!!')
end
if sum(y_dis_index) > round((gps_err_percentage/100)*N)
    gpsY_issue = true;
    if gpsX_issue == false
        disp('!!! Potential discrepancy in location measurement detected !!!')
    end
end
if sum(steer_dis_index) > round((steer_err_percentage/100)*N)
    steer_issue = true;
    disp('!!! Potential discrepancy in steer angle measurement detected !!!')
end

if ~(vel_issue || gpsX_issue || gpsY_issue || steer_issue)
    disp('No discrepencies detected!')
end


%% Plot results

figure()
subplot(2,1,1)
hold on
plot(standard_kalman.time(:,1),standard_kalman.data(:,7))
plot(vel_kalman.time(:,1),vel_kalman.data(:,7))
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Standard kalman filter','Filter not using velocity measurements')
title('Velocity comparison')
hold off
subplot(2,1,2)
hold on
plot(standard_kalman.time(:,1), vel_discrepancy)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Difference between estimations')
hold off



figure()
subplot(2,2,1:2)
hold on
plot(standard_kalman.data(:,1),standard_kalman.data(:,2))
plot(gps_kalman.data(:,1),gps_kalman.data(:,2))
plot(gps_kalman.data(1,1),gps_kalman.data(1,2),'o','Color','green')
plot(standard_kalman.data(1,1),standard_kalman.data(1,2),'o','Color','green')
title('Position comparison')
axis equal
xlabel('X pos')
ylabel('Y pos')
legend('Standard kalman filter','Filter not using GPS measurements','Start location')
hold off
subplot(2,2,3)
hold on
plot(standard_kalman.time(:,1), x_coord_discrepancy)
xlabel('Time (s)')
ylabel('X pos')
title('Difference between X- estimations')
hold off
subplot(2,2,4)
hold on
plot(standard_kalman.time(:,1), y_coord_discrepancy)
xlabel('Time (s)')
ylabel('Y pos')
title('Difference between Y- estimations')
hold off


figure()
subplot(2,1,1)
hold on
plot(standard_kalman.time(:,1),rad2deg(standard_kalman.data(:,6)))
plot(steer_kalman.time(:,1),rad2deg(steer_kalman.data(:,6)))
title('Steer angle comparison')
xlabel('Time (s)')
ylabel('Steer angle (degrees)')
legend('Standard kalman filter','Filter not using steering angle encoder measurements')
hold off
subplot(2,1,2)
hold on
plot(standard_kalman.time(:,1),rad2deg(steer_discrepancy))
title('Steer angle difference')
xlabel('Time (s)')
ylabel('Steer angle (degrees)')
hold off



% figure()
% subplot(2,1,1)
% hold on
% plot(standard_kalman.time(:,1),rad2deg(standard_kalman.data(:,4)))
% plot(steer_kalman.time(:,1),rad2deg(steer_kalman.data(:,4)))
% plot(steer_kalman.time(:,1),rad2deg(fake_roll))
% title('Roll angle comparison')
% xlabel('Time (s)')
% ylabel('Roll angle (degrees)')
% legend('Standard kalman filter','Filter not using steering angle encoder measurements')
% hold off
% subplot(2,1,2)
% hold on
% plot(standard_kalman.time(:,1),rad2deg(standard_kalman.data(:,4)-steer_kalman.data(:,4)))
% title('Roll angle difference')
% xlabel('Time (s)')
% ylabel('Roll angle (degrees)')
% hold off
% 
% figure()
% subplot(2,1,1)
% hold on
% plot(standard_kalman.time(:,1),rad2deg(standard_kalman.data(:,5)))
% plot(steer_kalman.time(:,1),rad2deg(steer_kalman.data(:,5)))
% title('Roll angle rate comparison')
% xlabel('Time (s)')
% ylabel('Roll angle (degrees)')
% legend('Standard kalman filter','Filter not using steering angle encoder measurements')
% hold off
% subplot(2,1,2)
% hold on
% plot(standard_kalman.time(:,1),rad2deg(standard_kalman.data(:,5)-steer_kalman.data(:,5)))
% title('Roll angle rate difference')
% xlabel('Time (s)')
% ylabel('Roll angle (degrees)')
% hold off

