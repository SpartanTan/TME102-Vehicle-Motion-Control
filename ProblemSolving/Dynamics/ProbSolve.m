%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem 1

%constants
m=2e3;
c=40;
I=2e3;
vx=20;
Lf=1.2;
Lr=1.4;

Cf=Lr/(Lr+Lf)*c*m*9.81;
Cr=Lf/(Lr+Lf)*c*m*9.81;


% From slide 12 on first lecture

A=[-(Cf+Cr)/m/vx (-Cf*Lf+Cr*Lr)/m/vx-vx; ...
    (-Cf*Lf+Cr*Lr)/I/vx, -(Cf*Lf^2+Cr*Lr^2)/I/vx]
B=[Cf/m;Cf*Lf/I]

C=[0 1] % The second state is the yaw rate
D=0;

% Form a state space object
ssG=ss(A,B,C,D);

% convert to trasfer function

G=tf(ssG)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem 3

bode(G)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem 4

vx=20;
A=[-(Cf+Cr)/m/vx (-Cf*Lf+Cr*Lr)/m/vx-vx; ...
    (-Cf*Lf+Cr*Lr)/I/vx, -(Cf*Lf^2+Cr*Lr^2)/I/vx];
eig(A)
% Form a state space object
ssG20=ss(A,B,C,D);

vx=10;
A=[-(Cf+Cr)/m/vx (-Cf*Lf+Cr*Lr)/m/vx-vx; ...
    (-Cf*Lf+Cr*Lr)/I/vx, -(Cf*Lf^2+Cr*Lr^2)/I/vx];
eig(A)
% Form a state space object
ssG10=ss(A,B,C,D);

vx=30;
A=[-(Cf+Cr)/m/vx (-Cf*Lf+Cr*Lr)/m/vx-vx; ...
    (-Cf*Lf+Cr*Lr)/I/vx, -(Cf*Lf^2+Cr*Lr^2)/I/vx];
eig(A)
% Form a state space object
ssG30=ss(A,B,C,D);


bode(ssG10,ssG20,ssG30)
legend('10','20','30')