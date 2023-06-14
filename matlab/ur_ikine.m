function q = ur_ikine(T,d,a)
% UR机械臂(三轴平行构型)逆运动学求解(解析形式)
%   alpha = [pi/2,0,0,-pi/2,pi/2,0]
%   offset = [0,pi/2,-pi/2,0,pi/2,0]

q = zeros(1,6);

% (1)
R06 = T(1:3,1:3);
o6 = T(1:3,4);
z6 = T(1:3,3);

% (2)
z5 = z6;
o5 = o6-d(6)*z5;

% handle singular input
d234 = d(2)+d(3)+d(4);
if norm(o5)>(a(2)+a(3))*0.85
  o5 = o5/norm(o5)*(a(2)+a(3))*0.85;
elseif (norm(o5(1:2))<(abs(d234))*2)
  o5(1) = o5(1)/norm(o5(1:2))*abs(d234)*2;
  o5(2) = o5(2)/norm(o5(1:2))*abs(d234)*2;
end

% (3)
xo5 = o5(1);
yo5 = o5(2);
if xo5^2+yo5^2<d234^2
  return; % no solution
end
if d234==0
  q(1) = atan2(yo5,xo5);
else
  q(1) = acos(abs(d234)/sqrt(xo5^2+yo5^2))+atan2(yo5,xo5)+abs(d234)/d234*pi/2;
end

% (4)
R01 = [cos(q(1)),0,sin(q(1));sin(q(1)),0,-cos(q(1));0,1,0];
z1 = R01(1:3,3);
z4_ = cross(z1,z5);
z4 = z4_/norm(z4_);
% if norm(z4_)>1e-8
%   z4 = z4_/norm(z4_);
% end
o4 = o5-d(5)*z4;

% (5)
o1 = [0,0,d(1)]';
p14 = o4-o1;
if norm(p14)^2>(a(2)+a(3))^2+d234^2 || norm(p14)^2<(a(2)-a(3))^2+d234^2
  return; % no solution
end
phi = acos((a(2)^2+a(3)^2+d234^2-norm(p14)^2)/(2*a(2)*a(3)));
q(3) = phi-pi/2;

% (6)
y1 = [0,0,1]';
x1 = cross(y1,z1);
gamma = asin(a(3)*sin(phi)/sqrt(norm(p14)^2-d234^2));
q(2) = atan2(y1'*p14,x1'*p14)+gamma-pi/2;

% (7)
y4 = -z1;
x4 = cross(y4,z4);
R04 = [x4,y4,z4];
R13 = [cos(q(2)+q(3)),-sin(q(2)+q(3)),0; ...
       sin(q(2)+q(3)),cos(q(2)+q(3)),0; ...
       0,0,1];
R34 = (R01*R13)'*R04;
q(4) = atan2(R34(2,1),R34(1,1));

if mod(q(4),2*pi)>pi/2 && mod(q(4),2*pi)<3*pi/2
  z4 = -z4;
  % goto (4)
  o4 = o5-d(5)*z4;

  % (5)
  o1 = [0,0,d(1)]';
  p14 = o4-o1;
  if norm(p14)^2>(a(2)+a(3))^2+d234^2 || norm(p14)^2<(a(2)-a(3))^2+d234^2
    return; % no solution
  end
  phi = acos((a(2)^2+a(3)^2+d234^2-norm(p14)^2)/(2*a(2)*a(3)));
  q(3) = phi-pi/2;
  
  % (6)
  y1 = [0,0,1]';
  x1 = cross(y1,z1);
  gamma = asin(a(3)*sin(phi)/sqrt(norm(p14)^2-d234^2));
  q(2) = atan2(y1'*p14,x1'*p14)+gamma-pi/2;
  
  % (7)
  y4 = -z1;
  x4 = cross(y4,z4);
  R04 = [x4,y4,z4];
  R13 = [cos(q(2)+q(3)),-sin(q(2)+q(3)),0; ...
         sin(q(2)+q(3)),cos(q(2)+q(3)),0; ...
         0,0,1];
  R34 = (R01*R13)'*R04;
  q(4) = atan2(R34(2,1),R34(1,1));
end

% (8)
y5 = z4;
x5 = cross(y5,z5);
R05 = [x5,y5,z5];
R45 = R04'*R05;
q(5) = atan2(R45(2,1),R45(1,1))-pi/2;

% (9)
R56 = R05'*R06;
q(6) = atan2(R56(2,1),R56(1,1));

end
