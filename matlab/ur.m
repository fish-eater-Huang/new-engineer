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

% forward kinematic
T = UR.fkine(q);

% jacobian matrix
J = UR.jacob0(q);

% innverse kinematic
q_ikine = UR.ikine(T);
q_ikine_self = ur_ikine(T.T,UR.d,UR.a);

% plot
figure(1); view(3);
UR.plot(q);
% UR.teach;

% % 操作控制误差控制
% kv = 0.01;
% kw = 0.01;
% Td = UR.fkine(zeros(1,6));
% ve = kv*(Td.t-T.t);
% Te = Td.T*T.T^-1;
% Re = Te(1:3,1:3); % Rd = Re*R
% [theta,vec] = tr2angvec(Re);
% we = kw*theta*vec';
% dq = (J\[ve;we])*100;
