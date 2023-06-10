clear; clc; close all;

% UR robot
L(1) = Link([0,0,0,pi/2]);
L(2) = Link([0,0.5,5,0]);
L(3) = Link([0,-0.5,6,0]);
L(4) = Link([0,-1,0,-pi/2]);
L(5) = Link([0,1,0,pi/2]);
L(6) = Link([0,1,0,0]);
UR = SerialLink(L,"name","UR");

% offset
UR.offset = [0,pi/2,-pi/2,0,pi/2,0];

% joint variable
% q = [0,0,0,0,0,0];
q = [0.1,0.2,0.3,0.4,0.5,0.6];
q_D1 = [0,0,0,0,0,0];
q_D2 = [0,0,0,0,0,0];

% forward kinematic
T = UR.fkine(q);

% jacobian matrix
J = UR.jacob0(q);

% inverse kinematic
q_ikine = UR.ikine(T);
q_ikine_self = ur_ikine(T.T,UR.d,UR.a);

% inverse dynamic
torq = UR.rne(q,q_D1,q_D2);

% plot
figure(1); view(3);
UR.plot(q);
% UR.teach;
