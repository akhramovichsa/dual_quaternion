function angl_rt = orb_getAngularRate(Orbit)
global MU_EARTH 
  p = orb_getParam(Orbit);
  r2 = orb_getRadius(Orbit)^2;
  angl_rt = sqrt( MU_EARTH * p)/r2;
end
