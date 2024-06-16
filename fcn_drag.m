function [D] = fcn_drag(m,zacc,xacc,T,AOA,density,airspeed,vehicle)

if vehicle=='linus'
    CLmindrag = 0.000000;
    Cd0 = 0.02477653 -0.005;
    e = 0.36179696;
    AR = 17.05;
    S = 0.378460;
end

%calculate drag as a function of CL and TAS 
        L = m .* ((zacc .* cosd(AOA)) + (xacc .* sind(AOA))) - T.*sind(AOA);
        CL = 2.*L./(S.*density.*(airspeed.^2));
        Cd = Cd0 + (CL - CLmindrag).^2./(pi.*e.*AR);
        D = Cd.*S.*density.*(airspeed.^2)./2;
