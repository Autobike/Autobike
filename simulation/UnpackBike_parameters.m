function [hh,lr,lf,lambda,cc,mm,h_imu,Tt,r_wheel]=UnpackBike_parameters(bike_params)
%[hh,lr,lf,lambda,cc,mm,h_imu,Tt]=UnpackBike_parameters(bike_params)
%
% UnpackBike_parameters unpacks the individual bike parameters from the
% data structure bike_params
%
% See LoadBikeParameters for details of bike_params
% Tt is a matrix describing the position and direction of the IMU

hh = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
cc = bike_params.c_mod;
mm= bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;
r_wheel=bike_params.r_wheel;

% Correction IMU   TODO: explain, probably for ajusting of how the IMU is
% mounted on the bike
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

Tt = Rz*Ry*Rx;  