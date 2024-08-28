function [] = SaveInCSV(matrixmat,test_curve)
% 
%SaveInCSV(matrixmat,test_curve)
%
%

filename_matrix = 'matrixmat.csv'; % filename for bike parameters
filename_trajectory = 'trajectorymat.csv'; % filename for trajectory

csvwrite(filename_matrix, matrixmat); % Write the matrix to the CSV file

csvwrite(filename_trajectory, test_curve); % Write the matrix to the CSV file

end