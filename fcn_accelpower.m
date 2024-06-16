function accelpowreq = fcn_accelpower(m,xacc,g,pitch,airspeed)


accelpowreq = m .* (xacc-g .*sind(pitch) ) .* airspeed;

