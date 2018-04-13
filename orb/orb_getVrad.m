function Vrd =  orb_getVrad(Orbit) 
global MU_EARTH 
 Vrd = sqrt( MU_EARTH/ orb_getParam(Orbit))* Orbit(2) * sin( orb_getTrueAnom(Orbit)); 
end