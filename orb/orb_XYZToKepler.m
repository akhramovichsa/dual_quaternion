function Orbit = orb_XYZToKepler(rx,ry,rz,Vx,Vy,Vz)
global MU_EARTH
C = [ry*Vz - rz*Vy 
     rz*Vx - rx*Vz
     rx*Vy - ry*Vx
];

ra = norm([rx ry rz]);

F = [ C(3)*Vy - C(2)*Vz - MU_EARTH * rx/ra  
      C(1)*Vz - C(3)*Vx - MU_EARTH * ry/ra
      C(2)*Vx - C(1)*Vy - MU_EARTH * rz/ra
];

p = (norm(C)^2)/MU_EARTH;
e = norm(F)/MU_EARTH;
Rp = p/(1+e);
i = acos(C(3)/norm(C));
 if i ~= 0 
     if C(1)*sin(i)/norm(C) >= 0
         Om = acos(-C(2)/(norm(C)*sin(i))); 
     else Om = 2*pi-acos(-C(2)/(norm(C)*sin(i))); 
     end
     if rz >= 0 
         u = acos((rx * cos(Om) + ry * sin(Om))/ ra );
     else u = 2*pi - acos((rx * cos(Om) + ry * sin(Om))/ ra );
     end
 else
    if ry >= 0 
        u = acos(rx/ra);
    else u = 2*pi - acos(rx/ra);
        Om = u;
    end
 end
 
 F = F/norm(F);

if e > 0 
    if cos(i) ~= 0 
        S_Om = (-F(1)*sin(Om) + F(2)*cos(Om))/cos(i);
        C_Om = F(1)*cos(Om) + F(2)*sin(Om);
    else
        S_Om = F(3);
        C_Om = F(3)/sin(u);
    end
    if S_Om >= 0 
        w = acos(C_Om);
    else w = 2*pi - acos(C_Om);
    end
else w = u;
end

Orbit = [Rp, e, u, w, i, Om];

for i = 3:6 
    Orbit(i) = rad2deg(Orbit(i));
end


end