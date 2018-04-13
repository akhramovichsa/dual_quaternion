function [rx,ry,rz,Vx,Vy,Vz] = orb_KeplerToXYZ (Orbit)
 for i = 3:6 
    Orbit(i) = deg2rad(Orbit(i));
 end 
 
O = [Orbit(3), Orbit(6), Orbit(5)];
 Sn = sin(O);
 Cn = cos(O);
 
ra = orb_getRadius(Orbit);
 
rx  = ( Cn(1) * Cn(2) - Sn(1) * Sn(2) * Cn(3) ) * ra;
ry  = ( Cn(1) * Sn(2) + Sn(1) * Cn(2) * Cn(3) ) * ra;
rz  = Sn(1) * Sn(3) * ra;

Vr_r = orb_getVrad(Orbit)/ra;
Rw = orb_getAngularRate(Orbit)*ra;

Vx  = Vr_r * rx - Rw *(Sn(1) * Cn(2) + Cn(1) * Sn(2) * Cn(3) );
Vy  = Vr_r * ry - Rw *(Sn(1) * Sn(2) - Cn(1) * Cn(2) * Cn(3) );
Vz  = Vr_r * rz + Rw * Cn(1) * Sn(3);
end