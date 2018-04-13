function ra = orb_getRadius (Orbit)
  ra = orb_getParam(Orbit)/(1 + Orbit(2) * cos(orb_getTrueAnom(Orbit)));
end


