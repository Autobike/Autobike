function [initialization_new] = referenceTest(test_curve,hor_dis,Ts,vv)
%[initialization_new] = referenceTest(test_curve,hor_dis,Ts,v)
%
% Check for some possible problems with the reference. The purpose of the
% function is to give warnings to help catching up misstakes at an early
% stage.

%

[nn] = size(test_curve,1); % number of samples in the reference

%% Minimun number of reference points, unclear test
if nn<(hor_dis/Ts)
    disp('Warning: Not enough reference points');
end

%% Initialization errors
%initialization_new = initial_pose;
initialization_new = test_curve(1,1:3);

% if test_curve(1,1) ~= initial_pose(1) || test_curve(1,2) ~= initial_pose(2) || test_curve(1,3) ~= initial_pose(3)
%     disp('Warning: initialization has been corrected');
%     initialization_new(1) = test_curve(1,1);
%     initialization_new(2) = test_curve(1,2);
%     initialization_new(3) = test_curve(1,3);
%     if sqrt((initial_pose(1)-test_curve(1,1))^2+(initial_pose(2)-test_curve(2,2))^2) >= 2
%         disp('Warning: Error in Initial position larger than 2 meters');
%     end
%     if abs(initial_pose(3)-test_curve(3,3)) >= pi/6
%         disp('Warning: Error in initial heading larger than 30 degree');
%     end
% end
        
%% Density of datapoints
Min_density_distance = 10;
index = 0;
for i = 1:nn-1
    dist = sqrt((test_curve(i,1)-test_curve(i+1,1))^2+(test_curve(i,2)-test_curve(i+1,2))^2);
    if dist > Min_density_distance
        index = [index i];
    end
end    
if length(index) ~= 1
    message_dens = ['Warning: Distance between some ref points are more than ',num2str(Min_density_distance),'m, index: '];
    disp(message_dens)
    disp(index(2:end))
end
  
%% Warns for sharp turns which are hard to reach for the bike    
%it warns when a turn of >90 degrees will be made within ts/v
%Determine amount of datapoints which should be added
%Include dpsiref in this matlab script
% 
% turn_distance = 2/(ref_dis*10);
% max_turn = pi/6;
% index = 0;
% Warningturn = rad2deg(max_turn);
% 
% if turn_distance > 1
%     for i = 1:n-1
%         dpsiref(i) = abs(test_curve(i,3)-test_curve(i+1,3));
%     end
% 
%     for i = 1:1:length(dpsiref)-turn_distance
%         tot_dpsiref = sum(dpsiref(i:i+turn_distance));
%         if tot_dpsiref >= max_turn   
%             index = [index i];
%         end
%     end
% 
%     if length(index) ~= 1
%         message_turn = ['Warning: An unrealistic sharp turn has been detected (>',num2str(Warningturn),' degree), index:'];
%         disp(message_turn)
%         disp(index(2:end))
%     end
% end


%% Check travelled distance between sampling points, and compare this with the distance 
%% between points on the reference trajectory. The difference should not be too large.
max1=10;  % tune the test with these constants
min1=10;

bike_dis=vv*Ts; % travelled distance between sampling instances

% distance between reference points
tra_dis=sqrt(sum([test_curve(1:end-1,1)-test_curve(2:end,1) test_curve(1:end-1,2)-test_curve(2:end,2)].^2,2));

if any(bike_dis>tra_dis) %if bike_dis is larger a lot than tra_dis, it's difficult to track reference trajectory, display warning
    disp('Warning: Sparse reference trajectory, several time samples between reference trajectory samples.');
end
if max(tra_dis)>max1*bike_dis,
    disp(join(['Most extreme case ' num2str(max(tra_dis)/bike_dis) ' time samples between reference trajectory samples.']));
end
if min(tra_dis)<min1*bike_dis,
    disp(join(['Most extreme case ' num2str(bike_dis/min(tra_dis)) ' reference trajectory samples between two time samples.']));
end
    disp(join(['Avarage ' num2str(sum(tra_dis,1)/(bike_dis*length(tra_dis))) ' reference trajectory for each time samples.']));


