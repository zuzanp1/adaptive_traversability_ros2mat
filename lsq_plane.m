function [x, xn, roughness] = lsq_plane(xyz)
%LSQ for plane fitting into the noisy data
% ux + vy + w = z (z is a function of x and y)-> terrain map
%A = [x1 y1 1;
%        .
%        .
%     xn yn 1];
%b = [z1,...zn]';
%x = [u v w]'; slope-intersect form
x = zeros(3,1);
xn = zeros(4,1);
roughness = zeros;
if isempty(xyz), return; end;
    
N = size(xyz,1);

A = [xyz(:,[1 2]), ones(size(xyz,1),1)];
b = xyz(:,end);

x = ((A'*A)\A')*b;

%compute the average roughness parameter per pixel
e = A*x -b;
roughness = (e'*e)/N;

%ax + by + cz + d = 0 s.t: a^2+b^2+c^2=1 general form
c = 1/sqrt(1+x(1)^2 + x(2)^2);%positive UP
a = -x(1)*c;
b = -x(2)*c;
d = -x(3)*c;
xn = [a;b;c;d];

end