%clc
%clear all;
 
a1= 12.5;
a2 =25.5;
theta1 = pi/4;
theta2 = pi/8;
d3 =2;
 
%Initial position
ini_x = -9;
ini_y = 20;
ini_z = 0;
ini_q = [0.399    2.100 0];
% Time between two points in seconds
t = 0.05;
 
% Number of points along the path
pts = 485;
 
% Time segments for plotting graphs
tGraph  = 0:0.05:t*pts;
 
%Defining the robot
L1 = Link([theta1 0 a1 0 0]);
L2 = Link([theta2 0 a2 pi 0]);
L3 = Link([0 d3 0 0 1]);
L = SerialLink([L1 L2 L3]);
L.name = 'SCARA';
 
%Initializing Matrices for linear and angular velocity storage
linearVs = zeros(pts,3);
angularVs = zeros(pts,3);
SCARADrawn_XYZp = zeros(pts,3);
 
figure(1);
 
%Initial Pose
T = transl(ini_x, ini_y, ini_z);
qs = L.ikine(T, [0.399    2.100 0], [1 1 1 0 0 0])
plot(L ,qs,'workspace', [-pi pi -pi pi -5 5])
qs = qs';
 
for i = 1:pts
%Calculating the Jacobian for the current coordinate
T = transl(contour(i,1),contour(i,2),0);
qs = L.ikine(T, qs, [1 1 1 0 0 0]);
J = jacob0(L, qs);
 
Jxyz = J(1:3,:);
 
%Calculating the required angular velocities
qd = inv(Jxyz)*  [contour(i,3) contour(i,4) 0 ]';
angularVs(i,:) = qd';
 
xd = Jxyz*qd;
linearVs(i,:)=xd;
 
% Angular displacement
qs =  qs' + qd*t;
 
figure(1);
 
% Plot the robot
plot(L ,qs','workspace', [-pi pi -pi pi -5 5])
 
% XY Plot
Tfkine = L.fkine(qs');
p = transl(Tfkine);
figure(2);
hold on;
plot(p(1), p(2))
SCARADrawn_XYZp(i,:) = p';
 
end