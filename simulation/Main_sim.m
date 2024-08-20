%% clear the possible remnant on previous running
set(0,'defaulttextinterpreter','none');
dbclear all; %Remove breakpoints
clear;
close all;
clc;

%% Simulation Settings and Bike and General Parameters
% Gravitational Acceleration
    g = 9.81;
% Name of the model
    model = 'Main_bikesim';
% Simulation time
    sim_time = 400;
% Sampling Time
    Ts = 0.01; 
% First closest point selection in reference. Starts at 2 because the one 
% before closest is in the local reference as well
    ref_start_idx = 2; %end of page 64 of Lorenzo's thesis
% Horizon distance [m]
    hor_dis = 10; %tra cosa?

%Constant Speed [m/s]
    % v = 3;    

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options:'red', 'black', 'green', 'scooter' or 'plastic' 
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% bike model (for Simulink)
    bike_model = 1; % 1 = non-linear model || 2 = linear model
% 0 = Don't run test cases & save measurementdata in CSV || 1 = run test cases || 2 = generate ref yourself
    Run_tests = 0; 
% Take estimated states from a specific time if wanted (0 == initial conditions are set to zero || 1 == take from an online test)
    init = 0;
    time_start = 14.001; % what time do you want to take (if init==1)
% When you have bad GPS signal:
% Set indoor to 1 when you run the bike indoor or you have bad GPS signal
    indoor = 0; % used in Simulink
% Set badGPS=1 to make the GPS signal steady from the beginning of the sim.
    badGPS = 0; % used in Simulink
% Set compare_flag=1 if you want to compare two simulation results
    compare_flag = 0;
% Activate gain scheduling for system matrices and gains that depend on the
% velocity. Implemented on Kalman Filter and Heading dot contribution transfer function
    scheduling = 1;
% Activate the interpolation for the gain scheduling. Interpolation is
% always supposed to be used when scheduling = 1. 
    interpolation = 1;  % used in Simulink (you can only use interpolation if scheduling = 1)

%% Initial states
if init == 1

    % data_lab = readtable('Logging_data\Test_session_14_06\data_8.csv');
    data_lab = readtable('Logging_data\Test_session_27_06\data_15.csv');

    %Delete the data before reseting the trajectory and obtain X/Y position
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
    initial_state.x = 1; %why 1 and not 0?
    initial_state.y = 0;
    initial_state.heading = deg2rad(0);
    initial_pose=[initial_state.x; initial_state.y; initial_state.heading];
else
    disp('Bad initialization');
end

%% Reference trajectory generation
%SHAPE options:sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
type = 'circle';
% Distance between points
ref_dis = 0.5;
% Number of reference points
N = 80; 
% Scale (only for infinite and circle)
scale = 40; 

% [Xref,Yref,Psiref] = Trajectory(Run_tests);
% [Xref,Yref,Psiref,Vref,t]=Refgeneration({'x','y','v'},'AATraj.csv');

[Xref,Yref,Psiref,Vref,t]=Refgeneration({'x','y','v'},'AATrajCorrectedSpeed.csv');
% ref_traj = [Xref,Yref,Psiref,Vref];
% csvwrite("AATrajectory.csv",ref_traj);

Vref = Vref;
v_init=Vref(1); % needed for lqr, referenceTest, simulink>atateestimator



test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%% OWN TRAJECTORY
% if Run_tests == 2
%[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
% test_traj();
% data = fileread('trajectory.txt');
% test_curve=[Xref,Yref,Psiref];
% Nn = size(test_curve,1); % needed for simulink
% end

%% Reference test (warnings and initialization update)
if ((Run_tests == 0 || Run_tests == 2) && init == 0)

    Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v_init, ref_dis);
    
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

T = TransMatrix(bike_params);                                             

%% Disturbance Model
% 
% % Roll Reference  
% roll_ref_generation;%long time ago left by other students, it's helpless now but keep it
% 
% % Steering Rate State Perturbation
% pert_deltadot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_deltadot_state_fun = @(time)  -0.5*(time>10) && (ceil(mod(time/3,2)) == 1) &&(time<30);
% 
% % Roll Rate State Perturbation
% pert_phidot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_phidot_state_fun = @(time) cos(time)*(time>10 && time < 10.4);

%% Bike State-Space Model

% Continuous-Time Model

% % Controllable Canonical Form
%     A = [0 g/bike_params.h ; 1 0];
%     B = [1 ; 0];
%     C = [bike_params.a*v/(bike_params.b*bike_params.h) g*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+v^2/(bike_params.b*bike_params.h)];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];

% % Observable Canonical Form
%     A = [0 g/bike_params.h ; 1 0];
%     B = [g*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+(v^2./(bike_params.b*bike_params.h)-bike_params.a*bike_params.c*g/(bike_params.b*bike_params.h^2)).*sin(bike_params.lambda) ;
%         bike_params.a*v/(bike_params.b*bike_params.h).*sin(bike_params.lambda)];
%     C = [0 1];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];
% 
% % Linearized System
%     linearized_sys = ss(A,B,C,D);
% % Augmented System
%     fullstate_sys = ss(linearized_sys.A,linearized_sys.B,eye(size(linearized_sys.A)),0);
% % Discretized System
%     discretized_sys = c2d(linearized_sys,Ts);

%% Balancing Controller
%Remove I and D?
% Outer loop -- Roll Tracking
P_balancing_outer = 3.75;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3.5;
I_balancing_inner = 0;
D_balancing_inner = 0; 


%% Calculating gains and matrices which depend on velocity, based on velocity vector which is created below. 
if scheduling
    V_min= min(Vref(:));
    V_max= max(Vref(:));
else % If the speed is going to be a constant all the time, then v min and max are same and put them down here. Defaulted to 3 but
     % user can change it if desired.
    V_min= 3;
    V_max= 3;
end

V_stepSize=0.1;

V_n=ceil((V_max-V_min)/V_stepSize)+1;
V=linspace(V_min,V_max,V_n);

V=round(V,1);

K_GPS=zeros(V_n,7,7);
K_noGPS=zeros(V_n,7,7);


counter=zeros(V_n,1);
A_d=zeros(V_n,7,7);
B_d=zeros(V_n,7,1);
C=zeros(V_n,7,7);
D=zeros(V_n,7,1);

A_t=zeros(V_n,1);
B_t=zeros(V_n,1);
C_t=zeros(V_n,1);
D_t=zeros(V_n,1);

load('Q_and_R_backup_red_bike.mat');

format long
for i=1: V_n
    % Kalman filtering for both cases - with/without GPS - 
    [K_GPS(i,:,:),K_noGPS(i,:,:),counter,A_d(i,:,:),B_d(i,:,:),C(i,:,:),D(i,:,:)] = KalmanFilter(V(i),h,lr,lf,lambda,g,c,h_imu,Ts,Q,R);

    % Transfer function in heading in wrap traj
    num = 1;
    den = [lr/(lr+lf), V(i)/(lr+lf)];
    [A_t(i,:), B_t(i,:), C_t(i,:), D_t(i,:)] = tf2ss(num,den);
end
K_GPS=permute(K_GPS,[1,3,2]);
K_noGPS=permute(K_noGPS,[1,3,2]);
A_d=permute(A_d,[1,3,2]);
C=permute(C,[1,3,2]);

% Storing all the calculated matrices and gains.
GainsTable = table(V',K_GPS,K_noGPS,A_d,B_d,C,D, 'VariableNames', {'V','K_GPS','K_noGPS','A_d','B_d','C','D'});



%% The LQR controller
[k1,k2,e1_max,e2_max] = LQRcontroller(v_init,lr,lf);

%% Transfer function for heading in wrap traj
%feed forward trasfer function for d_psiref to steering reference (steering contribution for heading changes)

% Discretize the ss 
% % Used in Simulink
Ad_t = eye(1)+Ts*A_t;% A_t and B_t are calculated on gains table section above.
Bd_t = B_t*Ts;


%% Save matrix in XML/CSV
% matrixmat = [A_d; B_d'; C; D';K_GPS; K_noGPS]; 
% SaveInCSV(matrixmat,test_curve);

%% Start the Simulation
if Run_tests == 0 || Run_tests == 2
tic
try 
    Results = sim(model); % If no error occurs, MATLAB skips the catch.
    catch error_details %note: the model has runned for one time here
end
toc

% Simulation Messages and Warnings
% if Results.stop.Data(end) == 1
%     disp('Message: End of the trajectory has been reached');
% end

%% Plotting
% If you want to compare two different simulation results, then change the
% name of 'bikedata_sim_real_states.csv' and 'bikedata_sim_est.csv' to
% 'bikedata_sim_real_states_1.csv' and 'bikedata_sim_est_1.csv after
% running main_sim.m the first time and change compare_flag to 1 before
% running main_sim.m the second time.
PlottingResults(test_curve,Results,compare_flag);
end

%% Test cases for validation
TestCases(Run_tests,hor_dis,Ts,initial_pose);
%%
