clear; clc; close all;

% exchange station
arm_L(1) = Link("d",0,"a",0,"alpha",-pi/2,"m",0.83,"r",[0,0.003,0.049],"I",zeros(3));
arm_L(2) = Link("d",0,"a",0.266,"alpha",0,"m",1.222,"r",[-0.06,0,0.062],"I",zeros(3));
arm_L(3) = Link("d",0,"a",0,"alpha",-pi/2,"m",0.538,"r",[-0.004,0,0.012],"I",zeros(3));
arm_L(4) = Link("d",0.28,"a",0,"alpha",pi/2,"m",0.342,"r",[0,-0.327,0.035],"I",zeros(3));
arm_L(5) = Link("d",0,"a",0,"alpha",-pi/2,"m",0.162,"r",[0,-0.02,0],"I",zeros(3));
arm_L(6) = Link("d",0.07,"a",0,"alpha",0,"m",0.12,"r",[0,0,-0.015],"I",zeros(3));
arm = SerialLink(arm_L,"name","es");
arm.offset = [0,0,0,0,0,0];

% joint variable
% q = [0,0,0,0,0,0];
q = [0.1,0.2,0.3,0.4,0.5,0.6];
q_D1 = [0,0,0,0,0,0];
q_D2 = [0,0,0,0,0,0];

% forward kinematic
T = arm.fkine(q);

% jacobian matrix
J = arm.jacob0(q);

% inverse kinematic
q_ikine = arm.ikine(T);
q_ikine_self = p560_ikine(T.T,arm.d,arm.a);

% inverse dynamic
torq = arm.rne(q,q_D1,q_D2);

% plot
figure(1); view(3);
arm.plot(q);

%% 逆运动学测试
% t = 0:0.1:5;
% pt = zeros(3,size(t,2));
% qt = zeros(6,size(t,2));
% figure(2); view(3);
% for i = 1:size(t,2)
%   Rt = [0,0,1;1,0,0;0,1,0];
%   pt(:,i) = [0.6;0.25+0.6*sin(t(i));0.3];
%   Tt = [Rt,pt(:,i);zeros(1,3),1];
%   qt(:,i) = ur_ikine(Tt,es.d,es.a)';
%   es.plot(qt(:,i)');
% %   pause(0.1);
% end
% 
% figure(3);
% subplot(3,1,1); title("x");
% plot(t,pt(1,:),"lineWidth",1);
% subplot(3,1,2); title("y");
% plot(t,pt(2,:),"lineWidth",1);
% subplot(3,1,3); title("z");
% plot(t,pt(3,:),"lineWidth",1);
% 
% figure(4);
% subplot(6,1,1); title("q1");
% plot(t,qt(1,:),"lineWidth",1);
% subplot(6,1,2); title("q2");
% plot(t,qt(2,:),"lineWidth",1);
% subplot(6,1,3); title("q3");
% plot(t,qt(3,:),"lineWidth",1);
% subplot(6,1,4); title("q4");
% plot(t,qt(4,:),"lineWidth",1);
% subplot(6,1,5); title("q5");
% plot(t,qt(5,:),"lineWidth",1);
% subplot(6,1,6); title("q6");
% plot(t,qt(6,:),"lineWidth",1);
