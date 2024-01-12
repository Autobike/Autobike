%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

% Specify parameters to vary
pred_horiz_span = [130];
% Pf and Q weights are relative to R, which is kept constant at 1
% e1 and e2 spans for Pf and Q must have same lengths
Pf_e1_span = [3];
Pf_e2_span = [100];
Q_e1_span = [1]*1e-2;
Q_e2_span = [1]*1e-2;

%% Simulation Settings and Bike Parameters
% General Parameters

    % Gravitational Acceleration
        g = 9.81;
    % Name of the model
        model = 'Main_bikesim';
    % Simulation time
        sim_time = 400;

    % Sampling Time
        Ts = 0.01; 
    % First closest point selection in reference 
    % Starts at 2 because the one before closest is in the local reference as well
        ref_start_idx = 2;
    % Horizon distance [m]
        hor_dis = 10;
    % Constant Speed [m/s]
        v = 2.4;    

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options: 'red', 'black', 'green' and 'scooter'
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% bike model
    bike_model = 1; % 1 = non-linear model || 2 = linear model
% 0 = Don't run test cases & save measurementdata in CSV || 1 = run test cases || 2 = generate ref yourself
    Run_tests = 0; 
% Take estimated states from a specific time if wanted (0 == initial conditions are set to zero || 1 == take from an online test)
    init = 0;
    time_start = 14.001;         % what time do you want to take

%% Initial states

if init == 1

%     data_lab = readtable('Logging_data\Test_session_14_06\data_8.csv');
data_lab = readtable('Logging_data\Test_session_27_06\data_15.csv');

    % Delete the data before reseting the trajectory and obtain X/Y position
    reset_traj = find(data_lab.ResetTraj==1,1,'last');
    data_lab(1:reset_traj,:) = [];
    longitude0 = deg2rad(11);
    latitude0 = deg2rad(57);
    Earth_rad = 6371000.0;
    
    X = Earth_rad * (data_lab.LongGPS_deg_ - longitude0) * cos(latitude0);
    Y = Earth_rad * (data_lab.LatGPS_deg_ - latitude0);

    % Obtain the relative time of the data
%     Y = round(X,N) 
    data_lab.Time = round((data_lab.Time_ms_- data_lab.Time_ms_(1))*0.001, 4);
    index = find(data_lab.Time == time_start);

    initial_state.roll = data_lab.StateEstimateRoll_rad_(index);
    initial_state.roll_rate = data_lab.StateEstimateRollrate_rad_s_(index);
    initial_state.steering = data_lab.StateEstimateDelta_rad_(index);
    initial_state_estimate.x = data_lab.StateEstimateX_m_(index) - X(1);
    initial_state_estimate.y = data_lab.StateEstimateY_m_(index) - Y(1);
    initial_state_estimate.heading = data_lab.StateEstimatePsi_rad_(index);

elseif init == 0
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
        initial_state.steering = deg2rad(0);
        initial_state.x = 0;
        initial_state.y = 0;
        initial_state.heading = deg2rad(0);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;
else
    disp('Bad initialization');
end

%% Reference trajectory generation

% SHAPE options: sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
type = 'sharp_turn';
% Distance between points
ref_dis = 0.5;
% Number# of reference points
N = 70;
% Scale (only for infinite and circle)
scale = 40;
% Angle [deg] (only for sharp turn)
angle = 60;

[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale,angle);

test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%% Reference test (warnings and initialization update)
if ((Run_tests == 2) && init == 0)

Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v, ref_dis);

% update initial states if offset is detected
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2);
initial_state.heading = Output_reference_test(3);
initial_state.heading = Psiref(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;
end

%% Unpacked bike_params
h = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
c = bike_params.c_mod;
m = bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;

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

%% Balancing Controller

% Outer loop -- Roll Tracking
P_balancing_outer = 1.3;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3;
I_balancing_inner = 0;
D_balancing_inner = 0;  

%% The LQR controller

% error model for LQR controller calculation
A_con=[0 v;0 0];
B_con=[lr*v/(lr+lf);v/(lr+lf)];

%  kk=0.000000010;
%  kk=2.581e-7;
%  kk=9e-7;
 kk=6e-7;
%  Q=kk*[100 0;0 1000];
 Q=kk*[1000 0;0 100];
 R=0.2;
%  R=0.5;
 [K,S,e] = lqr(A_con,B_con,Q,R);
 k1=K(1);
 k2=K(2);

e2_max=deg2rad(30);%Here is the e2_max we used to calculate e1_max
e1_max=abs(-k2*e2_max/k1);% k1,k2 has been known, so we can calculate e1_max

%% Transfer function for heading in wrap traj
%feed forward trasfer function for d_psiref to steering reference (steering contribution for heading changes)
num = 1;
den = [lr/(lr+lf), v/(lr+lf)];
[A_t, B_t, C_t, D_t] = tf2ss(num,den);

% Discretize the ss
Ad_t = (eye(size(A_t))+Ts*A_t);
Bd_t = B_t*Ts;

%% Kalman Filter

% % A matrix (linear bicycle model with constant velocity)
% % Est_States := [X Y psi phi phi_dot delta v]
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

% Load the Q and R matrices
load('Q_and_R_backup_red_bike.mat');

% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig] = idare(A_d',C1',Q,R,[],[]);
    eig1 = abs(eig);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 

% Kalman_gain excluding GPS
Kalman_gain2 = Kalman_gain1(4:7,3:7);
%% Start the Simulation
%% MPC Trajectory controller
foldername = "Simulations on " + string(datestr(now,30));
mkdir("batch_simulations/" + foldername)
filename_trajectory = "batch_simulations/" + foldername + '/trajectorymat.csv'; % Specify the filename
csvwrite(filename_trajectory, test_curve); % Write the matrix to the CSV file
disp("Running " + (length(pred_horiz_span)*length(Pf_e1_span)*length(Q_e1_span)+1) + " simulations.")
simN = 1;
for i=1:length(pred_horiz_span)
    for j=1:length(Pf_e1_span)
        for k=1:length(Q_e1_span)
            disp("Running simulation " + simN)
            max_permissible_e1=100;
            max_permissible_e2=100;
            a = lr;
            b = lr+lf;
            
            N_outer=pred_horiz_span(i); %prediction horizont
            %penalty matrices 
            Q_outer=[Q_e1_span(k) 0; 0 Q_e2_span(k)]*1e-2;  % penalty on state deviation
            Pf_outer=[Pf_e1_span(j) 0; 0 Pf_e2_span(j)];  % penalty on final prediction step, i.e. "how important to reach"
            R_outer=1;          % penalty on control signal 
            
            % x=[e1 e2]'
            A_outer=[0 v;0 0];
            B_outer=[a*v/b;v/b];
            
            C_outer=eye(2);
            D_outer=zeros(1,2)';
            
            sys_outer = ss(A_outer,B_outer,C_outer,D_outer);
            
            % Discretization
            sys_dis_outer = c2d(sys_outer,Ts);
            Ad_outer = sys_dis_outer.A;
            Bd_outer = sys_dis_outer.B;
            Cd_outer = sys_dis_outer.C;
            Dd_outer = sys_dis_outer.D;
            
            %mpc 
            stateOfConstraint_outer=[1 0;
                               -1 0;
                               0 1;
                               0 -1];
            stateConstraintVal_outer=[max_permissible_e1;
                                max_permissible_e1;
                                max_permissible_e2;
                                max_permissible_e2];
            inputConstraint_outer=[1;
                            -1];
            
            inputConstraintVal_outer=deg2rad(20);%max_permissible_str-deg2rad(10); % harder constraint on outer?
            
            [rowInputConstraint_outer,colInputConstraint_outer]=size(inputConstraint_outer);
            [rowStateOfConstraint_outer,colStateOfConstraint_outer]=size(stateOfConstraint_outer);
            
            %Obs. constraints have the form Fx +Gu <=h, different from MPC course...
            %Size of F:
            %   - Cols: cols in x constr. times prediction steps (N) 
            %   - Rows: rows in x constr. times N + rows in u constr.
            %           times N
            F_outer=[kron([eye(N_outer)],stateOfConstraint_outer);
                zeros(rowInputConstraint_outer*N_outer,colStateOfConstraint_outer*N_outer)];
            %Size of G:
            %   - Cols: cols in u constr. times N
            %   - Rows: rows in x constr. times N + rows in u constr.
            %           times N
            G_outer=[zeros(N_outer*rowStateOfConstraint_outer,N_outer);kron(eye(N_outer),[1; -1])];
            
            %Size of h_mpc:
            %   -Cols: 1
            %   -Rows: rows in x constr. times N + rows in u constr.
            %           times N
            h_mpc_outer=[kron([ones(1*N_outer,1)],stateConstraintVal_outer);
                ones(N_outer*rowInputConstraint_outer,1)*inputConstraintVal_outer];
            
            mpc_outer_params=struct();
            mpc_outer_params.Ad=Ad_outer;
            mpc_outer_params.Bd=Bd_outer;
            mpc_outer_params.Cd=Cd_outer;
            mpc_outer_params.Dd=Dd_outer;
            mpc_outer_params.N=N_outer;
            mpc_outer_params.Q=Q_outer;
            mpc_outer_params.Pf=Pf_outer;
            mpc_outer_params.R=R_outer;
            mpc_outer_params.F=F_outer;
            mpc_outer_params.G=G_outer;
            mpc_outer_params.h=h_mpc_outer;
            
            
            tic
            try Results = sim(model);
                catch error_details %note: the model has runned for one time here
            end
            toc
            % save the states for offline analysis
            bikedata_simulation_bikestates = array2table([Results.bike_states.Time Results.bike_states.Data, Results.stop.Data, Results.ids.Data]);
            filename_simulation_bikestates = "batch_simulations/" + foldername + "/bikedata_simulation_real_states"+string(i)+string(j)+string(k)+".csv"; % Specify the filename
            bikedata_simulation_bikestates.Properties.VariableNames(1:10) = {'Time', 'X', 'Y', 'Psi', 'Roll', 'Rollrate', 'Delta', 'Velocity', 'Stop', 'Total_idx'};
            writetable(bikedata_simulation_bikestates ,filename_simulation_bikestates);
            
            % save measurement data and estimations for offline analysis
            bikedata_simulation = array2table([Results.estimated_states.Time Results.closest_point.Data Results.closestpoint_heading.Data ...
                Results.delta_e1.Data Results.delta_e2.Data Results.delta_psi.Data Results.delta_ref.Data Results.e1.Data Results.e2.Data ...
                Results.error1.Data Results.error2.Data Results.estimated_states.Data Results.measurements.Data Results.refRoll.Data ...
                Results.ref_states.Data Results.roll_ref.Data Results.steerrate_input.Data]);
            filename_simulation= "batch_simulations/" + foldername + "/bikedata_simulation"+string(i)+string(j)+string(k)+".csv"; % Specify the filename
            bikedata_simulation.Properties.VariableNames(1:35) = {'Time', 'Closestpoint_idx', 'Closestpoint_heading_idx', 'delta_e1', 'delta_e2', 'delta_psi',...
                'delta_ref', 'e1', 'e2', 'error1', 'error2', 'X_est', 'Y_est', 'Psi_est', 'Roll_est', 'Rollrate_est', 'Delta_est', 'Velocity_est', 'X_meas',...
                'Y_meas', 'Psi_meas', 'Roll_meas', 'Rollrate_meas', 'Delta_meas', 'Velocity_meas', 'refRoll', 'X_ref', 'Y_ref', 'Psi_ref', 'Roll_ref',...
                'Rollrate_ref', 'Delta_ref', 'Velocity_ref', 'roll_ref', 'Steerrate_input'};
            writetable(bikedata_simulation,filename_simulation);
            simN = simN + 1;
        end
    end
end
%% Simulate LQR for comparison
disp("Running simulation " + simN + ", LQR comparison")
model = 'Main_bikesim_old';
% Open the Simulink Model
open([model '.slx']);
% Choose the solver
set_param(model,'AlgebraicLoopSolver','TrustRegion');

tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc
% save the states for offline analysis
bikedata_simulation_bikestates = array2table([Results.bike_states.Time Results.bike_states.Data, Results.stop.Data, Results.ids.Data, Results.closest_point.Data]);
filename_simulation_bikestates = "batch_simulations/" + foldername + "/bikedata_simulation_real_statesLQR.csv"; % Specify the filename
bikedata_simulation_bikestates.Properties.VariableNames(1:11) = {'Time', 'X', 'Y', 'Psi', 'Roll', 'Rollrate', 'Delta', 'Velocity', 'Stop', 'Closestpoint_idx', 'Total_idx'};
writetable(bikedata_simulation_bikestates ,filename_simulation_bikestates);

% save measurement data and estimations for offline analysis
bikedata_simulation = array2table([Results.estimated_states.Time Results.closestpoint_heading.Data ...
    Results.delta_e1.Data Results.delta_e2.Data Results.delta_psi.Data Results.delta_ref.Data Results.e1.Data Results.e2.Data ...
    Results.error1.Data Results.error2.Data Results.estimated_states.Data Results.measurements.Data Results.refRoll.Data ...
    Results.ref_states.Data Results.roll_ref.Data Results.steerrate_input.Data]);
filename_simulation= "batch_simulations/" + foldername + "/bikedata_simulationLQR.csv"; % Specify the filename
bikedata_simulation.Properties.VariableNames(1:34) = {'Time', 'Closestpoint_heading_idx', 'delta_e1', 'delta_e2', 'delta_psi',...
    'delta_ref', 'e1', 'e2', 'error1', 'error2', 'X_est', 'Y_est', 'Psi_est', 'Roll_est', 'Rollrate_est', 'Delta_est', 'Velocity_est', 'X_meas',...
    'Y_meas', 'Psi_meas', 'Roll_meas', 'Rollrate_meas', 'Delta_meas', 'Velocity_meas', 'refRoll', 'X_ref', 'Y_ref', 'Psi_ref', 'Roll_ref',...
    'Rollrate_ref', 'Delta_ref', 'Velocity_ref', 'roll_ref', 'Steerrate_input'};
writetable(bikedata_simulation,filename_simulation);
simN = simN + 1;

% Save simulation settings
sim_tbl = struct();
sim_tbl.pred_horiz_span = pred_horiz_span;
sim_tbl.Pf_e1_span = Pf_e1_span;
sim_tbl.Pf_e2_span = Pf_e2_span;
sim_tbl.Q_e1_span = Q_e1_span;
sim_tbl.Q_e2_span = Q_e2_span;

save("batch_simulations/" + foldername + "/tuning_setup.mat", "sim_tbl" )

%% Plot simulation results
disp('Done, plotting...')
Plot_MPC_tuning_results(Ts,foldername)
% Simulation Messages and Warnings
% if Results.stop.Data(end) == 1
%     disp('Message: End of the trajectory has been reached');
% end

