function [Xref,Yref,Psiref] = Trajectory(Run_tests)
% [Xref,Yref,Psiref] = Trajectory(Run_tests)
% 
% Reads a file containing a stored trajectory.
% This file seems unnecessary. (Jonas)

% traj = readtable('Traj_ref_test\trajectorymat_asta0_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_line.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_35.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_30.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_25.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_infinite_20.csv');
traj = readtable('Traj_ref_test\trajectorymat_asta0_turn_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_turn_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_lat_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_asta0_circle_3_l.csv');
% traj = readtable('trajectorymat.csv');

% traj = readtable('Traj_ref_test\trajectorymat_foot_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_parking_line.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_circle_3_r.csv');
% traj = readtable('EXJOBB\new_simulation\trajectorymat_foot_turn_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_turn_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_lat_right.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_lat_left.csv');
% traj = readtable('Traj_ref_test\trajectorymat_foot_circle_3_l.csv');
traj = table2array(traj);
traj = [traj(:,1)-traj(1,1), traj(:,2)-traj(1,2), traj(:,3)];
Xref = traj(3:end,1);
Yref = traj(3:end,2);
Psiref = traj(3:end,3);
end