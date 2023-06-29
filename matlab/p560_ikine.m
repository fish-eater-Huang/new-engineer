function q = p560_ikine(T,d,a,q0)
% UR机械臂(三轴共点构型)逆运动学求解(解析形式)
%   alpha = [-pi/2,0,-pi/2,pi/2,-pi/2,0]
%   offset = [0,0,0,0,0,0]

q = q0;

% (1)
R06 = T(1:3,1:3);
o6 = T(1:3,4);
z6 = T(1:3,3);

% (2)
z5 = z6;
o5 = o6-d(6)*z5;

% handle singular input
if norm(o5) > (a(2)+d(4))*0.9
  o5 = o5/norm(o5)*(a(2)+d(4))*0.9;
elseif (norm(o5(1:2)) < (a(2)+d(4))*0.05)
  o5(1) = o5(1)/norm(o5(1:2))*(a(2)+d(4))*0.05;
  o5(2) = o5(2)/norm(o5(1:2))*(a(2)+d(4))*0.05;
end

% (3)-theta1
xo5 = o5(1);
yo5 = o5(2);
q(1) = atan2(yo5,xo5);

% (4)-theta3
o1 = [0,0,0]';
p15 = o5-o1;
if norm(p15)^2 > (a(2)+d(4))^2 || norm(p15)^2 < (a(2)-d(4))^2
  return; % no solution
end
phi = acos((a(2)^2+d(4)^2+d(2)^2-norm(p15)^2)/(2*a(2)*d(4)));
q(3) = pi/2-phi;

% (5)-theta2
y1 = [0,0,-1]';
x1 = [cos(q(1)),sin(q(1)),0]';
gamma = asin(d(4)*sin(phi)/sqrt(norm(p15)^2-d(2)^2));
q(2) = atan2(y1'*p15,x1'*p15)-gamma;

% (6)
R01 = [cos(q(1)),0,-sin(q(1));sin(q(1)),0,cos(q(1));0,-1,0];
R12 = [cos(q(2)),-sin(q(2)),0;sin(q(2)),cos(q(2)),0;0,0,1];
R23 = [cos(q(3)),0,-sin(q(3));sin(q(3)),0,cos(q(3));0,-1,0];
R36 = (R01*R12*R23)'*R06;

% (7)-theta4
% handle wrist singularity
if R36(1,3)^2+R36(2,3)^2 > 1e-6
  theta4 = [atan2(R36(2,3),R36(1,3));atan2(R36(2,3),R36(1,3))+pi];
  % normalize theta4
  while theta4(1) > pi
    theta4(1) = theta4(1)-2*pi;
  end
  while theta4(1) < -pi
    theta4(1) = theta4(1)+2*pi;
  end
  while theta4(2) > pi
    theta4(2) = theta4(2)-2*pi;
  end
  while theta4(2) < -pi
    theta4(2) = theta4(2)+2*pi;
  end
  % select theta4
  if abs(theta4(1)-q(4)) < abs(theta4(2)-q(4))
    q(4) = theta4(1);
  else
    q(4) = theta4(2);
  end
end

% (8)-theta5
q(5) = atan2(-sign(cos(q(4)))*sign(R36(1,3))*norm(R36(1:2,3)),R36(3,3));

% (9)-theta6
R34 = [cos(q(4)),0,sin(q(4));sin(q(4)),0,-cos(q(4));0,1,0];
R45 = [cos(q(5)),0,-sin(q(5));sin(q(5)),0,cos(q(5));0,-1,0];
R56 = (R34*R45)'*R36;
q(6) = atan2(R56(2,1),R56(1,1));

end
