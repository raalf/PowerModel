function accelpowreq = fcn_accelpower(m,xacc,zacc,g,pitch,roll,airspeed,AOA)


accelpowreq = m .* ((xacc-g.*sind(pitch)).*cosd(AOA) +(zacc- g.*cosd(pitch).*cosd(roll)).*sind(AOA) ) .* airspeed;

