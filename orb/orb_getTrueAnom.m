function v = orb_getTrueAnom(Orbit) 
  v = Orbit(3) - Orbit(4);
  if v < 0 
      v = 2*pi + v; 
  end
end 