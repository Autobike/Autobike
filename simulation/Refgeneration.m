% This fucntion is used to generate the needed references in order to
% control the bike. The user need to attach the trajectory that the bike
% will follow as a csv file, inclouding three columns. 
% these theree columes have to be enough to output
% [X_ref,Y_ref,V_ref,Psi_ref,t] at each sample. 
% Any combination in form of [x,y,t] [v,t,psi] and [x,y,v] will work. 
% any other combination data will lead to an error.
% How to use: 
% - The user needs to provide the correct order of the data combination (in the given trajectory CSV file) as the first parameter.  
% - The user needs to provide the path directory of the CSV file as the
% second parameter. 
% The function output will be [X_ref,Y_ref,V_ref,Psi_ref,t]. 
% Examples:
% [x,y,psi,v,t]=Refgeneration({'x','y','t'},'Traj_ref_test\traj1.csv');
% [x,y,psi,v,t]=Refgeneration({'y','t','x'},'Traj_ref_test\traj2.csv');
% [x,y,psi,v,t]=Refgeneration({'v','t','psi'},'Traj_ref_test\traj3.csv');
% [x,y,psi,v,t]=Refgeneration({'x','y','v'},'Traj_ref_test\traj4.csv');

% How to get trajectorymat.csv which is uploaded to myRIO:
   % Create your trajectory as specified above. 
   % Use Refgeneration.m function to get the elements of trajectorymat.
   % Make a table in form of-->[Xref,Yref,Psiref,Vref].
   % Use csvwrite to save the table as trajectorymat.csv.

   % If the trajectory start heading angle needs to be changed from the
   % heading angle in the beginning of your trajectorymat.csv, rotate your
   % trajectory using rotate_traj.m. Then your file is ready to be uploaded
   % on myRIO.
   
% Authors: Armin Khorsandi & Ayman Alzein summer 2024

function [x,y,psi,v,t] = Refgeneration(layout,filePath)
    
    if isequal(layout, {'x', 'y', 't'})|| isequal(layout, {'t', 'y', 'x'})|| isequal(layout, {'x', 't', 'y'})|| isequal(layout, {'t', 'x', 'y'})|| isequal(layout, {'y', 't', 'x'})|| isequal(layout, {'y', 'x', 't'})
        i = 1;
    elseif isequal(layout, {'v', 'psi', 't'})|| isequal(layout, {'v', 't', 'psi'})|| isequal(layout, {'t', 'psi', 'v'})|| isequal(layout, {'t', 'v', 'psi'})|| isequal(layout, {'psi', 'v', 't'})|| isequal(layout, {'psi', 't', 'v'})
        i = 2;
    elseif isequal(layout, {'x', 'y', 'v'})|| isequal(layout, {'v', 'y', 'x'})|| isequal(layout, {'x', 'v', 'y'})|| isequal(layout, {'v', 'x', 'y'})|| isequal(layout, {'y', 'v', 'x'})|| isequal(layout, {'y', 'x', 'v'})
        i = 3;
    else
        error('Not a valid input layout. Please attach one csv file path with only three columns. For more info, please refer to the function description.')
    end

    table_ref = readtable(filePath);
    table_ref = renamevars(table_ref, 'Var1', layout(1));
    table_ref = renamevars(table_ref, 'Var2', layout(2));
    table_ref = renamevars(table_ref, 'Var3', layout(3));
    
    if i==1
        t=table_ref.t;
        x=table_ref.x;
        y=table_ref.y;
        psi=zeros(length(x),1);
        v=zeros(length(x),1);
        for i=1:length(x)-1
            dx=x(i+1)-x(i);
            dy=y(i+1)-y(i);
            dt=t(i+1)-t(i);
            psi(i)=atan2(dy,dx);
            v(i)=sqrt(dx^2+dy^2)/dt;
        end
        psi(end)=psi(end-1);
        v(end)=v(end-1);
    end
    if i==2
        t=table_ref.t;
        psi=table_ref.psi;
        v=table_ref.v;
        x=zeros(length(v),1);
        y=zeros(length(v),1);
        for i=1:length(v)-1
            dt=t(i+1)-t(i);
            x(i+1)= x(i)+(v(i)/dt)*cos(psi(i));
            y(i+1)= y(i)+(v(i)/dt)*sin(psi(i));
        end
    end
     if i==3
        v=table_ref.v;
        x=table_ref.x;
        y=table_ref.y;
        t=zeros(length(x),1);
        psi=zeros(length(x),1);
        for i=1:length(x)-1
            dx=x(i+1)-x(i);
            dy=y(i+1)-y(i);
            psi(i)=atan2(dy,dx);
            t(i+1)=sqrt(dx^2+dy^2)/v(i);
        end
        psi(end)=psi(end-1);
     end
end


