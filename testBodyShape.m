x0 = 0;
y0 = 0;
z0 = 0;

rx = 20;
ry = 20;
rz = 20;

n = 100;

s1 = 5;
s2 = 4;

% -pi <= theta <= pi is a row vector.
% -pi/2 <= phi <= pi/2 is a column vector.
theta = (-n:2:n)/n*pi;
phi = (-n:2:n)'/n*pi/2;
cosphi = cos(phi); cosphi(1) = 0; cosphi(n+1) = 0;
sintheta = sin(theta); sintheta(1) = 0; sintheta(n+1) = 0;

x = cosphi.^s1*cos(theta).^s2;
y = cosphi.^s1*sintheta.^s2;
z = sin(phi).^s1*ones(1,n+1);

    surf(x,y,z)
    xx = x; yy = y; zz = z;

% 
% t1 = linspace(-pi/2, pi/2, n);
% t2 = linspace(0, 2*pi, n);
% 
% s1 = 2;
% s2 = 2;
% 
% x= rx * cos(t1).^s1 .* cos(t2).^s2' + x0;
% y= ry * cos(t1).^s1 .* sin(t2).^s2' + y0;
% z= rz * sin(t1).^s1 .* ones(n,1) + z0;
% 
% surf(x, y, z)
% 
