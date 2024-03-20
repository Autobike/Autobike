function [] = SaveInCSV(matrixmat,test_curve)
filename_matrix = 'matrixmat.csv'; % Specify the filename
csvwrite(filename_matrix, matrixmat); % Write the matrix to the CSV file

filename_trajectory = 'trajectorymat.csv'; % Specify the filename
csvwrite(filename_trajectory, test_curve); % Write the matrix to the CSV file
end