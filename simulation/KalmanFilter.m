function [K_GPS,K_noGPS,counter,A_d,B_d,C,D] = KalmanFilter(v,h,lr,lf,lambda,g,c,h_imu,Ts,Q,R)
    % % A matrix (linear bicycle model with constant velocity)
    % % Est_States := [X Y psi phi phi_dot delta v]
    A = [0 0 0 0 0 0 1;
         0 0 v 0 0 v*(lr/(lf+lr))*sin(lambda) 0;
         0 0 0 0 0 (v/(lr+lf))*sin(lambda) 0;
         0 0 0 0 1 0 0;
         0 0 0 (g/h) 0 ((v^2/h)-(g*lr*c/(h^2*(lr+lf))))*sin(lambda) 0;
         0 0 0 0 0 0 0;
         0 0 0 0 0 0 0];
    
    % B matrix (linear bicycle model with constant velocity)
    B = [0 0 0 0 ((v*lr)/(h*(lr+lf))) 1 0]';
    
    % Including GPS
    C = [1 0 0 0 0 0 0;
         0 1 0 0 0 0 0;
         0 0 0 -g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
         0 0 0 0 1 0 0;
         0 0 0 0 0 (v)*sin(lambda)/(lr+lf) 0;
         0 0 0 0 0 1 0;
         0 0 0 0 0 0 1];
    
    D = [0 0 (-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';
    
    % % Excluding GPS
    % C2 = [(-g+((-h_imu*g)/h)) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2)+(v^2)*sin(lambda)/(lr+lf) 0;
    %       0 1 0 0;
    %       0 0 (v)*sin(lambda)/(lr+lf) 0;
    %       0 0 1 0;
    %       0 0 0 1];
    % 
    % D2 = [(-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';
    
    % Discretize the model
    A_d = (eye(size(A))+Ts*A);
    B_d = Ts*B;
    
    % Compute Kalman Gain including GPS
    [~,K_GPS,eig] = idare(A_d',C',Q,R,[],[]);
    eig1 = abs(eig);
    K_GPS = K_GPS';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag
    
    % Polish the kalman gain (values <10-5 are set to zero)
    for i = 1:size(K_GPS,1)
        for j = 1:size(K_GPS,2)
            if abs(K_GPS(i,j)) < 10^-5
                K_GPS(i,j) = 0;
            end
        end
    end 
    
    % Kalman gain excluding GPS
    R(1,1)=10^10*R(1,1); % X-coordinate
    R(2,2)=10^10*R(2,2); % Y-coordinate
    
    [~,K_noGPS,~] = idare(A_d',C',Q,R,[],[]);
    K_noGPS = K_noGPS';
    
    
    % Polish the kalman gain (values <10-5 are set to zero)
    for i = 1:size(K_noGPS,1)
        for j = 1:size(K_noGPS,2)
            if abs(K_noGPS(i,j)) < 10^-5
                K_noGPS(i,j) = 0;
            end
        end
    end 

end