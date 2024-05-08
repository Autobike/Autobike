clear
close all
clc

%% User has probably to modify these lines
traj=readtable('Traj_ref_test\trajectorymat_asta0_circle_3_l.csv');

% Set method=0 if heading is found by walking with the bike / method=1 if
% using a start and end point.
method = 2; 

% Method 0 (Read the psi from Kalman filter in LabVIEW)
bike_psi=-0.3+pi; %Actual heading of the bike or desired trajectory direction

% Method 1 (Read them from GPS Data in LabVIEW)
starting_long = 0; % Start point longitude
starting_lat = 0;  % Start point latitude
end_long = 1;      % End point longitude
end_lat = 1;       % End point latitude


%% Calculation of the rotated trajectory
if method == 1
    longitude0 = deg2rad(11); 
    latitude0 = deg2rad(57);
    Earth_rad = 6371000.0;  

    X_start = Earth_rad * (starting_long - longitude0) * cos(latitude0);
    Y_start = Earth_rad * (starting_lat - latitude0);
    
    X_end = Earth_rad * (end_long - longitude0) * cos(latitude0);
    Y_end = Earth_rad * (end_lat - latitude0);

    bike_psi = atan2(Y_end-Y_start,X_end-X_start);

elseif method == 0
else
    errordlg('The method can either be 0 or 1','Method Error')
    error('Wrong method value')
end

diff_angle = bike_psi - traj.Var3(1); %Angle difference between your heading and the heading of the first point
    
new_traj_psi= traj.Var3 + diff_angle;

new_traj_points = zeros(length(traj.Var1),2);
for i=1:length(traj.Var1)
    new_traj_points(i,:)=[traj.Var1(i) traj.Var2(i)]*[cos(diff_angle) sin(diff_angle); -sin(diff_angle) cos(diff_angle)];
end

new_traj= [new_traj_points new_traj_psi];



%% Plots
figure(1)
plot(traj.Var1,traj.Var2);
hold on
plot(traj.Var1(1),traj.Var2(1),'d');  %first point
hold on
plot(traj.Var1(10),traj.Var2(10),'d'); %tenth point
legend('Trajectory to rotate')

figure(2)
plot(new_traj_points(:,1),new_traj_points(:,2));
hold on 
plot(new_traj_points(1,1),new_traj_points(1,2),'d'); %first point
hold on 
plot(new_traj_points(10,1),new_traj_points(10,2),'d'); %tenth point
legend('New Trajectory')

% Save the file as a csv 
csv_file_path = 'Traj_ref_test\new_trajectoryfile.csv';  % Update this with your desired path
dlmwrite(csv_file_path, new_traj, 'delimiter', ',', 'precision', 10);


