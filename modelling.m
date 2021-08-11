clc
clear
close all

%% parameters
M = 2.001; %chassis mass
m = 3.031; %mass of wheels and shaft
g = 9.81; %gravitational acceleration
l = 808/1000; %length
b = 0.1; %viscuous friction coefficient estimate
I = 0.411;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

%% matrices
A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0];
%     0 0 1 0];
D = [0];
%     0];
 
%% controller
Q = [1     0     0     0
     0     0     0     0
     0     0     1     0
     0     0     0     0];
qq = 100*eye(4); 
R = 1;

sys = ss(A,B,C,D);
K_lqr = lqr(A,B,qq,R);

%eig(A)
%pole(sys)
Sc = ctrb(sys);
So = obsv(sys);

X0 = [0; 5*pi/180; 0;0];