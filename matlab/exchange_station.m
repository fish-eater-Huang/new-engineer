clear; clc; close all;

% exchange station
% es_L(1) = Link([0,0.352,0,pi/2]);
% es_L(2) = Link([0,0.117,0.4439,0]);
% es_L(3) = Link([0,-0.1218,0.4639,0]);
% es_L(4) = Link([0,-0.0475,0,-pi/2]);
% es_L(5) = Link([0,0.128,0,pi/2]);
% es_L(6) = Link([0,0.384,0,0]);
es_L(1) = Link("d",0.352,"a",0,"alpha",pi/2,"m",1.234,"r",[0,-0.017,0.018],"I",zeros(3));
es_L(2) = Link("d",0.117,"a",0.4439,"alpha",0,"m",2.326,"r",[-0.25,0,-0.033],"I",zeros(3));
es_L(3) = Link("d",-0.1218,"a",0.4639,"alpha",0,"m",2.182,"r",[-0.26,0,0.032],"I",zeros(3));
es_L(4) = Link("d",-0.0475,"a",0,"alpha",-pi/2,"m",0.648,"r",[-0.018,0,0.04],"I",zeros(3));
es_L(5) = Link("d",0.128,"a",0,"alpha",pi/2,"m",0.98,"r",[0,-0.022,0.085],"I",zeros(3));
es_L(6) = Link("d",0.384,"a",0,"alpha",0,"m",1.792,"r",[0,0,-0.123],"I",zeros(3));
es = SerialLink(es_L,"name","es");
es.offset = [0,pi/2,-pi/2,0,pi/2,0];

% joint variable
q = [0.3,1,-1,0.1,0.2,-0.1];
% q = [0,0,0,0,0,0];
q_D1 = [0,0,0,0,0,0];
q_D2 = [0,0,0,0,0,0];

% forward kinematic
T = es.fkine(q);

% jacobian matrix
J = es.jacob0(q);

% inverse kinematic
q_ikine = es.ikine(T);
q_ikine_self = ur_ikine(T.T,es.d,es.a);

% inverse dynamic
torq = es.rne(q,q_D1,q_D2);

% plot
figure(1); view(3);
es.plot(q);
% UR.teach;
