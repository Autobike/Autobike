%% 31/7 Jonas editerar och strukturerar koden för att få den mer lättanvänd. 
%

% Started: edit  main_sim.m to shorten code and to put functionality into
% functions which can be used also for simulation studies.

% Functions and scripts:
% Main_sim : script performing a simuloation, can be used as a start
%            example for new simulations.
% TestTrajectories: scipt showing how ReferenceGenerator and referenceTest
% can be used.

% [hh,lr,lf,lambda,cc,mm,h_imu,Tt]=UnpackBike_parameters(bike_params)
% [Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale)
% bike_params = LoadBikeParameters(bike)

% referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis)
%

% Use matlab to genererate data for test runs with bikes in Labview
% ReferenceGenerator.m can be used to generate trajectories
% Refgeneration.m convert reference trajectories to the correct format
% SaveInCSV.m store the trajectory in CSV-file, the format used to read in
%   the trajectory to LabView

% TODO
% modify Kalman_Q_R_tuner similar to main.m
% remove Kalmam_Q_R_tuner - just a misspelling
%
% Each function should have a short help text
% referenceTest needs more documentation, also the tests are unclear
% TestTrajectories needs to be edited to be useful