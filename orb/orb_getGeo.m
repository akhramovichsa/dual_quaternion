function [Lon, Lat] = orb_getGeo (r,t)
for i = 1:length(r)
 S = orb_getSiderealTime(t(i));   
 M = [cos(S) sin(S) 0;
     -sin(S) cos(S) 0;
       0       0    1];
 rt = M.*r(i,:); 
 rt = prod(rt);
 rm = norm(rt);
 Lat(i) = asin(r(i,3)/rm);
 w = sqrt(r(i,1)^2+r(i,2)^2);
 Ls = r(i,2)/ w;
 Lc = r(i,1)/ w;
 if Ls >= 0 
     Lon(i) = acos(Lc);
 else Lon(i) = -acos(Lc);
 end
end
Lon = rad2deg(Lon);
Lat = rad2deg(Lat);
end