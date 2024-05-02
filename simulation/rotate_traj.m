clear
close all
clc

bike_psi=5.1; %Actual heading of the bike or desired trajectory direction
traj=readtable('Traj_ref_test\trajectorymat_parking_line.csv');

diff_angle = bike_psi - traj.Var3(1); %Angle difference between your heading and the heading of the first point

new_traj_psi= traj.Var3 + diff_angle;

new_traj_points = zeros(length(traj.Var1),2);
for i=1:length(traj.Var1)
    new_traj_points(i,:)=[traj.Var1(i) traj.Var2(i)]*[cos(diff_angle) sin(diff_angle); -sin(diff_angle) cos(diff_angle)];
end

new_traj= [new_traj_points new_traj_psi];

plot(traj.Var1,traj.Var2);
figure
plot(new_traj_points(:,1),new_traj_points(:,2));


% Save the file as a csv 
csv_file_path = 'Traj_ref_test\new_trajectoryfile.csv';  % Update this with your desired path
dlmwrite(csv_file_path, new_traj, 'delimiter', ',', 'precision', 10);