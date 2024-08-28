% This fucntion is used to generate the needed references in order to
% control the bike. The user need to attach the trajectory that the bike
% will follow as a csv file, inclouding three columns. 
% these theree columes have to be enough to output
% [X_ref,Y_ref,V_ref,Psi_ref,t] at each sample. 
% [x,y,t] [v,t,psi] and [x,y,v] will work. 
% any other combination data will lead to an error.
% How to use: 
% - The user needs to provide the correct order of the data combination (in the given trajectory CSV file) as the first parameter.  
% - The user needs to provide the path directory of the CSV file as the
% second parameter. 
% The function output will be [X_ref,Y_ref,V_ref,Psi_ref,t]. 
% 
% [x,y,psi,v,t]=Refgeneration({'t' 'x','y'},'Traj_ref_test\traj1.csv');
% [x,y,psi,v,t]=Refgeneration({'t','v','psi'},'Traj_ref_test\traj3.csv');
% [x,y,psi,v,t]=Refgeneration({'x','y','v'},'Traj_ref_test\traj4.csv');
%
% or 
% [x,y,psi,v,t]=Refgeneration({'t' 'x','y'},matrix);
%
% where 'matrix' contains t, x, y  samples in one column each following
% that order.

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
% Modified by Jonas 2024-08-22

function [x,y,psi,vv,t] = Refgeneration(layout,filePath)
    
    % if isequal(layout, {'x', 'y', 't'})|| isequal(layout, {'t', 'y', 'x'})|| isequal(layout, {'x', 't', 'y'})|| isequal(layout, {'t', 'x', 'y'})|| isequal(layout, {'y', 't', 'x'})|| isequal(layout, {'y', 'x', 't'})
    %     nn = 1;
    % elseif isequal(layout, {'v', 'psi', 't'})|| isequal(layout, {'v', 't', 'psi'})|| isequal(layout, {'t', 'psi', 'v'})|| isequal(layout, {'t', 'v', 'psi'})|| isequal(layout, {'psi', 'v', 't'})|| isequal(layout, {'psi', 't', 'v'})
    %     nn = 2;
    % elseif isequal(layout, {'x', 'y', 'v'})|| isequal(layout, {'v', 'y', 'x'})|| isequal(layout, {'x', 'v', 'y'})|| isequal(layout, {'v', 'x', 'y'})|| isequal(layout, {'y', 'v', 'x'})|| isequal(layout, {'y', 'x', 'v'})
    %     nn = 3;
    % else
    %     error('Not a valid input layout. Please attach one csv file path with only three columns. For more info, please refer to the function description.')
    % end
    if isequal(layout, {'t', 'x', 'y'})
        nn = 1;
    elseif   isequal(layout, {'t', 'v', 'psi'})
        nn = 2;
    elseif isequal(layout, {'x', 'y', 'v'})
        nn = 3;
    else
        error('Not a valid input layout. Please attach one csv file path with only three columns. For more info, please refer to the function description.')
    end

    if ischar(filePath) % a file path containing the reference was submitted
        table_ref = readtable(filePath);
        table_ref = renamevars(table_ref, 'Var1', layout(1));
        table_ref = renamevars(table_ref, 'Var2', layout(2));
        table_ref = renamevars(table_ref, 'Var3', layout(3));
    else % a matrix with the reference was submitted
        table_ref.t=filePath(:,1);
        table_ref.x=filePath(:,2);
        table_ref.y=filePath(:,3);
        nn=1;
    end

    if nn==1
        x=table_ref.x;
        y=table_ref.y;
        t=table_ref.t;
        psi=zeros(length(x),1);
        vv=zeros(length(x),1);
        for inn=1:length(x)-1
            dx=x(inn+1)-x(inn);
            dy=y(inn+1)-y(inn);
            dt=t(inn+1)-t(inn);
            psi(inn)=atan2(dy,dx);
            vv(inn)=sqrt(dx^2+dy^2)/dt;
        end
        psi(end)=psi(end-1);
        vv(end)=vv(end-1);
    end
    if nn==2
        vv=table_ref.v;
        psi=table_ref.psi;
        t=table_ref.t;
        x=zeros(length(vv),1);
        y=zeros(length(vv),1);
        for inn=1:length(vv)-1
            dt=t(inn+1)-t(inn);
            x(inn+1)= x(inn)+(vv(inn)/dt)*cos(psi(inn));
            y(inn+1)= y(inn)+(vv(inn)/dt)*sin(psi(inn));
        end
    end
     if nn==3
        x=table_ref.x;
        y=table_ref.y;
        vv=table_ref.v;
        t=zeros(length(x),1);
        psi=zeros(length(x),1);
        for inn=1:length(x)-1
            dx=x(inn+1)-x(inn);
            dy=y(inn+1)-y(inn);
            psi(inn)=atan2(dy,dx);
            t(inn+1)=sqrt(dx^2+dy^2)/vv(inn);
        end
        psi(end)=psi(end-1);
     end
end


