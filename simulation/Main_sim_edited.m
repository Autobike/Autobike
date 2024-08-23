% This is a now - working file, but Jonas has made a lot of
% improvements/editing which should maybe be copied over to the working
% version.

% TODO describe the purpose of this file,
% what is done if no test is performed, what is done of tests are
% performed?

%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

%% Simulation Settings and Bike Parameters
% General Parameters

    % Constant Speed of bike [m/s] 
% TODO: this should probably be removed having Vref instead

        vv = 2.4;     % avoid symbols of only one letter


    % Simulation time
        sim_time = 400; % how is this paramter set? how does it relate to the length of the reference?

% Choose The Bike - Options: 'red', 'black', 'green' and 'scooter'
% parameters for each bike are set in LoadBikeParameters.m
    bike = 'red';


%% Balancing Controller  %TODO these parameters depend on the bike, and the speed, There should be /default) values 
%% in LoadBikeParameters.m which are read in here. The default can then be changed. Also, extend to speed dependent parameters.

% Outer loop -- Roll Tracking
P_balancing_outer = 3.75;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3.5;
I_balancing_inner = 0;
D_balancing_inner = 0;  

% 0 = Don't run test cases & save measurementdata in CSV || 1 = run test cases || 2 = generate ref yourself
    Run_tests = 0; 

    %% Reference trajectory generation

% SHAPE options: sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
type = 'circle';
% Distance between points
ref_dis = 0.5;
% Number# of reference points
N = 80; 
% Scale (only for infinite and circle)
scale = 40; 

[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);

% Calculating time vector given a constant speed
%TODO this should probably be changed, vv is used now
t_ref=cumsum([0;sqrt((Xref(1:end-1)-Xref(2:end)).^2+(Yref(1:end-1)-Yref(2:end)).^2)/vv]);

[Xref,Yref,Psiref,Vref]=Refgeneration({'t','x','y'},[t_ref, Xref,Yref]);

v_init=Vref(1); % needed for lqr, referenceTest, simulink>atateestimator

%% Initial state
% Take estimated states from a specific time if wanted 
% 0 => initial conditions are set to zero  
% 1 => take from recorded test data, interesting if you want to start the
% simulation from a specific time given the state of the bike in the test -
% good for evaluation of tests.

init = 0; % normal initial state for simulation, start on (0,0) direction of x-axis

if init==1 %indicate data record and at which time the state should be taken
    %     data_lab = readtable('Logging_data\Test_session_14_06\data_8.csv');
data_lab = readtable('Logging_data\Test_session_27_06\data_15.csv');
    time_start = 14.001;         % what time do you want to take    
end

%% End of setup for simulation, 

%% 
% Name of the Simulink model of bike system
    model = 'Main_bikesim';
% bike model, TODO: linear model is no up to date
    bike_model = 1; % 1 = non-linear model || 2 = linear model
% Gravitational Acceleration
    gg = 9.81; % avoid symbols of only one letter
% Sampling Time
    Ts = 0.01; 
% First closest point selection in reference 
% Starts at 2 because the one before closest is needed in the local reference as well
    ref_start_idx = 2;
% Horizon distance [m]  TODO explain
    hor_dis = 10;

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 


% %%  set initial states TODO: put into a separate function
% 
% if init == 1 % using recorded data
% 
%     % Delete the data before reseting the trajectory and obtain X/Y position
%     % TODO add small description of what is beeing done here
%     reset_traj = find(data_lab.ResetTraj==1,1,'last');
%     data_lab(1:reset_traj,:) = [];
%     longitude0 = deg2rad(11);
%     latitude0 = deg2rad(57);
%     Earth_rad = 6371000.0;
% 
%     X = Earth_rad * (data_lab.LongGPS_deg_ - longitude0) * cos(latitude0);
%     Y = Earth_rad * (data_lab.LatGPS_deg_ - latitude0);
% 
%     % Obtain the relative time of the data
% %     Y = round(X,N) 
%     data_lab.Time = round((data_lab.Time_ms_- data_lab.Time_ms_(1))*0.001, 4);
%     index = find(data_lab.Time == time_start);
% 
%     initial_state.roll = data_lab.StateEstimateRoll_rad_(index);
%     initial_state.roll_rate = data_lab.StateEstimateRollrate_rad_s_(index);
%     initial_state.steering = data_lab.StateEstimateDelta_rad_(index);
%     initial_state_estimate.x = data_lab.StateEstimateX_m_(index) - X(1);
%     initial_state_estimate.y = data_lab.StateEstimateY_m_(index) - Y(1);
%     initial_state_estimate.heading = data_lab.StateEstimatePsi_rad_(index);
% 
% elseif init == 0 % normal initial state for simulation, start on (0,0) direction of x-axis
%         initial_state.roll = deg2rad(0);
%         initial_state.roll_rate = deg2rad(0);
%         initial_state.steering = deg2rad(0);
%         initial_state.x = 1;
%         initial_state.y = 0;
%         initial_state.heading = deg2rad(0);
%         initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
% else
%     disp('Not suported value of init'); return;
% end
% 
% %% Reference trajectory generation
% 
% % SHAPE options: sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
% type = 'circle';
% % Distance between points
% ref_dis = 0.5;
% % Number# of reference points
% N = 80; 
% % Scale (only for infinite and circle, changes the distance between points....)
% scale = 40; 
% 
% [Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);

% Use a generated trajectory from file
% traj = readtable('Traj_ref_test\trajectorymat_asta0_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_line.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_35.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_30.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_25.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_20.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_turn_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_turn_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_lat_left.csv');
traj = readtable('Traj_ref_test\trajectorymat_asta0_circle_3_l.csv');
% traj = readtable('trajectorymat.csv');

% traj = readtable('Traj_ref_test\trajectorymat_foot_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_parking_line.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_circle_3_r.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_turn_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_turn_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_lat_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_circle_3_l.csv');
traj = table2array(traj);
traj = [traj(:,1)-traj(1,1), traj(:,2)-traj(1,2), traj(:,3)];
Xref = traj(3:end,1);
Yref = traj(3:end,2);
Psiref = traj(3:end,3);

test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%% OWN TRAJECTORY
% if Run_tests == 2
% test_trajectory();
% data = fileread('trajectory.txt');
% test_curve=[Xref,Yref,Psiref];
% Nn = size(test_curve,1); % needed for simulink
% end

%% Reference test (warnings and initialization update)
%if ((Run_tests == 0 || Run_tests == 2) && init == 0)
    
    Output_reference_test = referenceTest(test_curve,hor_dis,Ts,vv);
    
    % set initial state
    initial_state.x = Output_reference_test(1);
    initial_state.y = Output_reference_test(2);
    initial_state.heading = Output_reference_test(3);
    %initial_state.heading = Psiref(1);
    initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
    initial_state.roll = deg2rad(0);
    initial_state.roll_rate = deg2rad(0);
    initial_state.steering = deg2rad(0);
    initial_state_estimate = initial_state; % set the initial estimate to be the true state
%end

%% Unpacked bike_params
[hh,lr,lf,lambda,cc,mm,h_imu,Tt]=UnpackBike_parameters(bike_params);
                                                     

%% Disturbance Model
% 
% 
% % Steering Rate State Perturbation
% pert_deltadot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_deltadot_state_fun = @(time)  -0.5*(time>10) && (ceil(mod(time/3,2)) == 1) &&(time<30);
% 
% % Roll Rate State Perturbation
% pert_phidot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_phidot_state_fun = @(time) cos(time)*(time>10 && time < 10.4);

%% Bike balancing State-Space Model [roll roll_rate] = [phi phi_dot]

A = [0 1;
(gg/hh) 0];

% Continuous-Time Model
% 
% % Controllable Canonical Form
%     A = [0 gg/bike_params.h ; 1 0]; % lr=a, lf=b-a
%     B = [1 ; 0];
%     C = [bike_params.lr*vv/(bike_params.b*bike_params.h) gg*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+v^2/(bike_params.b*bike_params.h)];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];

% % Observable Canonical Form
%     A = [0 gg/bike_params.h ; 1 0];
%     B = [gg*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+(vv^2./(bike_params.b*bike_params.h)-bike_params.a*bike_params.c*gg/(bike_params.b*bike_params.h^2)).*sin(bike_params.lambda) ;
%         bike_params.a*v/(bike_params.b*bike_params.h).*sin(bike_params.lambda)];
%     C = [0 1];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];
% 
% Linearized System
%    linearized_sys = ss(A,B,C,D);
% % Augmented System
%     fullstate_sys = ss(linearized_sys.A,linearized_sys.B,eye(size(linearized_sys.A)),0);
% % Discretized System
%     discretized_sys = c2d(linearized_sys,Ts);
linearized_sys=0; % Symbol needed by simulink but not used at the moment


%% The LQR controller reference XXXXX

% error model for LQR controller calculation
A_con=[0 vv;0 0];
B_con=[lr*vv/(lr+lf);vv/(lr+lf)];

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

e2_max=deg2rad(30);% Here is the e2_max we used to calculate e1_max
e1_max=abs(-k2*e2_max/k1); % k1,k2 has been known, so we can calculate e1_max

% e2_max=deg2rad(30);
% e1_max=abs(1000);

%% Transfer function for heading in wrap traj
%feed forward trasfer function for d_psiref to steering reference (steering contribution for heading changes)
num = 1;
den = [lr/(lr+lf), vv/(lr+lf)];
[A_t, B_t, C_t, D_t] = tf2ss(num,den);

% Discretize the ss
Ad_t = (eye(size(A_t))+Ts*A_t);
Bd_t = B_t*Ts;

%% Kalman Filter

% % A matrix (linear bicycle model with constant velocity)
% % Est_States := [X Y psi phi phi_dot delta vv]
% % Outputs  [x Y 
A = [0 0 0 0 0 0 1;
     0 0 vv 0 0 vv*(lr/(lf+lr))*sin(lambda) 0;
     0 0 0 0 0 (vv/(lr+lf))*sin(lambda) 0;
     0 0 0 0 1 0 0;
     0 0 0 (gg/hh) 0 ((vv^2/hh)-(gg*lr*cc/(hh^2*(lr+lf))))*sin(lambda) 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0];

% B matrix (linear bicycle model with constant velocity)
B = [0 0 0 0 ((vv*lr)/(hh*(lr+lf))) 1 0]';

% Including GPS
C1 = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 -gg+((-h_imu*gg)/hh) 0 (-h_imu*(hh*vv^2-(gg*lr*cc)))*sin(lambda)/((lr+lf)*hh^2) + (vv^2)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 (vv)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];

D1 = [0 0 (-h_imu*lr*vv)/((lr+lf)*hh) 0 0 0 0]';

% Excluding GPS
C2 = [(-gg+((-h_imu*gg)/hh)) 0 (-h_imu*(hh*vv^2-(gg*lr*cc)))*sin(lambda)/((lr+lf)*hh^2)+(vv^2)*sin(lambda)/(lr+lf) 0;
      0 1 0 0;
      0 0 (vv)*sin(lambda)/(lr+lf) 0;
      0 0 1 0;
      0 0 0 1];

D2 = [(-h_imu*lr*vv)/((lr+lf)*hh) 0 0 0 0]';

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


%% Save matrix in XML/CSV

matrixmat = [A_d; B_d'; C1; D1';Kalman_gain1];

filename_matrix = 'matrixmat.csv'; % Specify the filename
csvwrite(filename_matrix, matrixmat); % Write the matrix to the CSV file

filename_trajectory = 'trajectorymat.csv'; % Specify the filename
csvwrite(filename_trajectory, test_curve); % Write the matrix to the CSV file


tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

% Simulation Messages and Warnings
% if Results.stop.Data(end) == 1
%     disp('Message: End of the trajectory has been reached');
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following is interesting in case we want to test different Kalman
% filteres on the same data

% save the states for offline kalman
bikedata_simulation_bikestates = array2table([Results.bike_states.Time Results.bike_states.Data]);
filename_simulation_bikestates = 'bikedata_simulation_real_states.csv'; % Specify the filename
bikedata_simulation_bikestates.Properties.VariableNames(1:8) = {'Time', 'X', 'Y', 'Psi', 'Roll', 'Rollrate', 'Delta', 'Velocity'};
writetable(bikedata_simulation_bikestates ,filename_simulation_bikestates);

% save measurement data and estimations for offline kalman
bikedata_simulation = array2table([Results.estimated_states.Time Results.estimated_states.Data Results.delta_e1.Data Results.delta_e2.Data ...
Results.delta_psi.Data Results.delta_ref.Data Results.roll_ref.Data Results.e1.Data Results.e2.Data]);
filename_simulation= 'bikedata_simulation.csv'; % Specify the filename
bikedata_simulation.Properties.VariableNames(1:15) = {'Time', 'X_estimated', 'Y_estimated', 'Psi_estimated', 'Roll_estimated', 'Rollrate_estimated', 'Delta_estimated', 'Velocity_estimated','delta_e1','delta_e2','delta_psi','delta_ref','ref_roll','error1','error2'};
writetable(bikedata_simulation,filename_simulation);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plotting
%name of the plot
Tnumber = 'No test case: General simulation run';
        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results);


