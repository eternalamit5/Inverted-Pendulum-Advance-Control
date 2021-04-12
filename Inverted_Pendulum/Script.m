%sample r a t e
Ts = 0.001 ; %s
% g r a v i t y a c c e l e r a t i o n
g = 9.81 ; % m/ s ˆ2
% l e n g t h o f pendulum :
l = 0.5 ; % m
% model o f motor b e h a v i o u r : PT1
T1 = 0.0395 ; % s

A=[0 1 0 0; 0 -1/T1 0 0; 0 0 0 1; 0 3/(2*l*T1) 3*g/(2*l) 0]; % Define the systemmatrix
B=[0;1/T1;0;-1/T1]; % Define the imputmatrix
C=[0 1 0 0; 0 0 0 1]; % Define the outputmatrix

Q_st=ctrb(A,B); % Calculate the controllability matrix
Q_be=obsv(A,C); % Calculate the observability matrix

[h,b]=size(A); % Calculate the size of the systemmatrix

if rank(Q_st)==h  % Check the full rank of the controllability matrix
    disp('controllable')
else
    disp('not observable')
end
    
if rank(Q_be)==h    % Check the full rank of the observability matrix
    disp('observable')
else
    disp('not observable')
end

v=[1,1,10,1];  % Define the weighting factor
Q=diag(v);     % Define the weighting matrix Q
R=1;           % Define the weighting matrix R
[K_R,S,e] = lqr(A,B,Q,R);  % Calculate the feedback matrix with LQR

p=[-10,-11,-12,-13]; % Define the poles 
K_p = place(A,B,p); % Calculate the feedback matrix with pole placement




