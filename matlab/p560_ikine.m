function q = p560_ikine(T,d,a)
% UR机械臂(三轴共点构型)逆运动学求解(解析形式)
%   alpha = [-pi/2,0,-pi/2,pi/2,-pi/2,0]
%   offset = [0,0,0,0,0,0]

q = zeros(1,6);

% (1)
R06 = T(1:3,1:3);
o6 = T(1:3,4);
z6 = T(1:3,3);

% (2)
z5 = z6;
o5 = o6-d(6)*z5;

% handle singular input
if norm(o5)>(a(2)+d(4))*0.9
  o5 = o5/norm(o5)*(a(2)+d(4))*0.9;
elseif (norm(o5(1:2))<(a(2)+d(4))*0.05)
  o5(1) = o5(1)/norm(o5(1:2))*(a(2)+d(4))*0.05;
  o5(2) = o5(2)/norm(o5(1:2))*(a(2)+d(4))*0.05;
end

% (3)
xo5 = o5(1);
yo5 = o5(2);
q(1) = atan2(yo5,xo5);

% (4)
o1 = [0,0,0]';
p15 = o5-o1;
if norm(p15)^2>(a(2)+d(4))^2 || norm(p15)^2<(a(2)-d(4))^2
  return; % no solution
end
phi = acos((a(2)^2+d(4)^2+d(2)^2-norm(p15)^2)/(2*a(2)*d(4)));
q(3) = pi/2-phi;

% (5)
y1 = [0,0,-1]';
x1 = [cos(q(1)),sin(q(1)),0]';
gamma = asin(d(4)*sin(phi)/sqrt(norm(p15)^2-d(2)^2));
q(2) = atan2(y1'*p15,x1'*p15)-gamma;

% (6)
R01 = [cos(q(1)),0,-sin(q(1));sin(q(1)),0,cos(q(1));0,-1,0];
R12 = [cos(q(2)),-sin(q(2)),0;sin(q(2)),cos(q(2)),0;0,0,1];
R23 = [cos(q(3)),0,-sin(q(3));sin(q(3)),0,cos(q(3));0,-1,0];
R36 = (R01*R12*R23)'*R06;
% if abs(R36(1,3))>1e-8
q(5) = atan2(-R36(1,3)/abs(R36(1,3))*norm(R36(1:2,3)),R36(3,3));
q(4) = atan2(-R36(2,3)/sin(q(5)),-R36(1,3)/sin(q(5)));
q(6) = atan2(-R36(3,2)/sin(q(5)),R36(3,1)/sin(q(5)));
% end

end
