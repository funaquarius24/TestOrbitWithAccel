function [r,v] = oe2rv(a,e,i,lan,w,nu)
% Reference: Bate, Mueller & White, Fundamentals of Astrodynamics Sec 2.5

mu = 398600.4405*1e9; % m^3 s^-2

% Express r and v in perifocal system
cnu = cosd(nu);
snu = sind(nu);
p = a*(1 - e^2);
r = p/(1 + e*cnu);
r_peri = [r*cnu ; r*snu ; 0];
v_peri = sqrt(mu/p)*[-snu ; e + cnu ; 0];

% Tranform into Geocentric Equatorial frame
clan = cosd(lan);
slan = sind(lan);
cw = cosd(w);
sw = sind(w);
ci = cosd(i);
si = sind(i);
R = [ clan*cw-slan*sw*ci  ,  -clan*sw-slan*cw*ci   ,    slan*si; ...
    slan*cw+clan*sw*ci  ,   -slan*sw+clan*cw*ci  ,   -clan*si; ...
    sw*si                  ,   cw*si                   ,   ci];
r = R*r_peri;
v = R*v_peri;
end