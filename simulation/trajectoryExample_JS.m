clear all
close all
clc


%% Section1: Define coordinates of points
% Coordinates of first point
lat1 = 57.782806; % Latitude of the first point in degrees
lon1 = 12.771923; % Longitude of the first point in degrees
% Coordinates of the second point
lat2 = 57.781885; % Latitude of the second point in degrees
lon2 = 12.772653; % Longitude of the second point in degrees

% Convert latitude and longitude to radians
lat1_rad = deg2rad(lat1);
lon1_rad = deg2rad(lon1);
lat2_rad = deg2rad(lat2);
lon2_rad = deg2rad(lon2);

% Point1 in format [x y]
Point1 = [Earth_radius * (lon1_rad - longitude0) * cos(latitude0) Earth_radius * (lat1_rad - latitude0)];
% Point2 in format [x y]
Point2 = [Earth_radius * (lon2_rad - longitude0) * cos(latitude0) Earth_radius * (lat2_rad - latitude0)];

