clear; clc; close all;

% arm
arm_L(1) = Link("d",0,"a",0,"alpha",-pi/2,"m",0.586,"r",[0,0.056,0.025],"I",zeros(3));
arm_L(2) = Link("d",0,"a",0.48,"alpha",0,"m",5.73,"r",[-0.248,0,0.071],"I",zeros(3));
arm_L(3) = Link("d",0,"a",0,"alpha",-pi/2,"m",1.334,"r",[0.002,0.01,0.101],"I",zeros(3));
arm_L(4) = Link("d",0.5125,"a",0,"alpha",pi/2,"m",1.447,"r",[-0.008,-0.128,0],"I",zeros(3));
arm_L(5) = Link("d",0,"a",0,"alpha",-pi/2,"m",0.105,"r",[0,0,0.012],"I",zeros(3));
arm_L(6) = Link("d",0.025,"a",0,"alpha",0,"m",0,"r",[0,0,0],"I",zeros(3));
arm = SerialLink(arm_L,"name","arm");
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
q_ikine_self = p560_ikine(T.T,arm.d,arm.a,zeros(6,1));

% inverse dynamic
torq = arm.rne(q,q_D1,q_D2);

% plot
figure(1); view(3);
arm.plot(q);

%% 逆运动学测试
t = 0:0.1:5;
pt = zeros(3,size(t,2));
qt = zeros(6,size(t,2));
figure(2); view(3);
for i = 1:size(t,2)
  Rt = [0,0,1;0,-1,0;1,0,0];
  pt(:,i) = [0.4;0;0.7*sin(t(i))];
  Tt = [Rt,pt(:,i);zeros(1,3),1];
  if i == 1
    qt(:,i) = p560_ikine(Tt,arm.d,arm.a,zeros(6,1));
  else
    qt(:,i) = p560_ikine(Tt,arm.d,arm.a,qt(:,i-1));
  end  
  arm.plot(qt(:,i)');
end

figure(3);
subplot(3,1,1); title("x");
plot(t,pt(1,:),"lineWidth",1);
subplot(3,1,2); title("y");
plot(t,pt(2,:),"lineWidth",1);
subplot(3,1,3); title("z");
plot(t,pt(3,:),"lineWidth",1);

figure(4);
subplot(6,1,1); title("q1");
plot(t,qt(1,:),"lineWidth",1);
subplot(6,1,2); title("q2");
plot(t,qt(2,:),"lineWidth",1);
subplot(6,1,3); title("q3");
plot(t,qt(3,:),"lineWidth",1);
subplot(6,1,4); title("q4");
plot(t,qt(4,:),"lineWidth",1);
subplot(6,1,5); title("q5");
plot(t,qt(5,:),"lineWidth",1);
subplot(6,1,6); title("q6");
plot(t,qt(6,:),"lineWidth",1);
