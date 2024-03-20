function [k1,k2,e1_max,e2_max] = LQRcontroller(v,lr,lf)
% error model for LQR controller calculation
% STATES := [e1; e2]
A_con=[0 v;0 0];
B_con=[lr*v/(lr+lf);v/(lr+lf)];

%  kk=0.000000010;
%  kk=2.581e-7;
%  kk=9e-7;
kk=6e-7;
%  Q=kk*[100 0;0 1000];
Q=kk*[1000 0;0 100];
R=0.2;
%  R=0.5;
[K,S,e] = lqr(A_con,B_con,Q,R);
k1=K(1);
k2=K(2);

e2_max=deg2rad(30);%Here is the e2_max we used to calculate e1_max
e1_max=abs(-k2*e2_max/k1);% k1,k2 has been known, so we can calculate e1_max

% e2_max=deg2rad(30);
% e1_max=abs(1000);
end