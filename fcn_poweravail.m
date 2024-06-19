function [powavail,Thrust] = fcn_poweravail(RPM,propd,density,airspeed,AOA,vehicle,prop)

% find Advance ratio J = V/(nD)
n = RPM./60;
J = airspeed ./ (n .* propd);

% CALCULATING CT
ct=fcn_ct(J,RPM,prop);

Thrust = (ct.*n.*n.*(propd).^4.*density ) ;

% Power Avail
powavail = (Thrust) .* airspeed .* cosd(AOA);
end

