% whats happening at 0RPM?
% do we need to tune these numbers live?
% needs tuning: mass, drag polar, to have no thermal when its calm
% needs the prop brake

%%tlog load
dialect = mavlinkdialect("ardupilotmega.xml");
logimport = mavlinktlog('2024-06-24 20-11-00.tlog',dialect);
% MESSAGE.GLOBAL_POSITION_INT=mavlinksub(gcsNode,uavClient,'GLOBAL_POSITION_INT');
% MESSAGE.SYSTEM_TIME=mavlinksub(gcsNode,uavClient,'SYSTEM_TIME');
%
% % MESSAGE.BATTERY_STATUS=mavlinksub(gcsNode,uavClient,'BATTERY_STATUS');
%
%
% MESSAGE.RAW_IMU=mavlinksub(gcsNode, uavClient, 'RAW_IMU');
% MESSAGE.VFR_HUD=mavlinksub(gcsNode, uavClient, 'VFR_HUD');
% MESSAGE.LOCALPOSITION_NED=mavlinksub(gcsNode, uavClient, 'LOCAL_POSITION_NED');
% MESSAGE.SCALED_PRESSURE=mavlinksub(gcsNode,uavClient,'SCALED_PRESSURE');
% MESSAGE.RPM=mavlinksub(gcsNode,uavClient,'RPM');
% MESSAGE.ATTITUDE=mavlinksub(gcsNode,uavClient,'ATTITUDE');
%%


vehicle = 'linus';

if strcmp(vehicle,'linus')

m = 1.793;

propd = 0.2794; %0.4699
prop = '11x7';
elseif strcmp(vehicle,'jsbrascal')

m = 13/2.2;

propd = 0.457; %0.4699
prop = '18x8';

end



msgs = readmsg(logimport, 'MessageName', 'VFR_HUD');%,'Time',[2000,3428]);
VFR_HUD = msgs.Messages{1};
[Time,timeidx]= unique(VFR_HUD.Time);
airspeed = double(VFR_HUD.airspeed(timeidx));

clear msgs
try
    msgs = readmsg(logimport, 'MessageName', 'GPS_RAW_INT');
    GLOBAL_POSITION_INT = msgs.Messages{1};
catch
    msgs = readmsg(logimport, 'MessageName', 'GPS2_RAW');
    GLOBAL_POSITION_INT = msgs.Messages{1};
end
[tempTime,temptimeidx]= unique(GLOBAL_POSITION_INT.Time);
lat = interp1(tempTime,double(GLOBAL_POSITION_INT.lat(temptimeidx)),Time)./1e7;
lng = interp1(tempTime,double(GLOBAL_POSITION_INT.lon(temptimeidx)),Time)./1e7;

clear msgs
msgs = readmsg(logimport, 'MessageName', 'RAW_IMU');
RAW_IMU = msgs.Messages{1};
[tempTime,temptimeidx]= unique(RAW_IMU.Time);
xacc = interp1(tempTime,double(RAW_IMU.xacc(temptimeidx)),Time)./ 100;
zacc = -interp1(tempTime,double(RAW_IMU.zacc(temptimeidx)),Time)./ 100;


clear msgs
msgs = readmsg(logimport, 'MessageName', 'LOCAL_POSITION_NED');
LOCAL_POSITION_NED = msgs.Messages{1};
[tempTime,temptimeidx]= unique(LOCAL_POSITION_NED.Time);
roc = -interp1(tempTime,double(LOCAL_POSITION_NED.vz(temptimeidx)),Time);
alt = -interp1(tempTime,double(LOCAL_POSITION_NED.z(temptimeidx)),Time);
%
clear msgs
msgs = readmsg(logimport, 'MessageName', 'SCALED_PRESSURE2');
SCALED_PRESSURE2 = msgs.Messages{1};
[tempTime,temptimeidx]= unique(SCALED_PRESSURE2.Time);
temperature = interp1(tempTime,double(SCALED_PRESSURE2.temperature_press_diff(temptimeidx)),Time)./100;
pressure = interp1(tempTime,double(SCALED_PRESSURE2.press_abs(temptimeidx)),Time).*100;


clear msgs
msgs = readmsg(logimport, 'MessageName', 'RPM');
RPM = msgs.Messages{1};
[tempTime,temptimeidx]= unique(RPM.Time);
rpm = interp1(tempTime+seconds(-0),double(RPM.rpm1(temptimeidx)),Time);

clear msgs
msgs = readmsg(logimport, 'MessageName', 'ATTITUDE');
ATTITUDE = msgs.Messages{1};
[tempTime,temptimeidx]= unique(ATTITUDE.Time);
pitch = interp1(tempTime,double(ATTITUDE.pitch(temptimeidx)),Time).*180./pi;
roll = interp1(tempTime,double(ATTITUDE.roll(temptimeidx)),Time).*180./pi;


clear msgs
msgs = readmsg(logimport, "MessageName", "AOA_SSA",'SystemID',1,'ComponentID',1);
AOA_SSA = msgs.Messages{1};
[tempTime,temptimeidx]= unique(AOA_SSA.Time);
AOA = interp1(tempTime,double(AOA_SSA.AOA(temptimeidx)),Time);

% SYNCFMT.RPM.rpm1 = rpm;
% SYNCFMT.ARSP.Airspeed = airspeed;
% SYNCFMT.IMU.AccZ = zacc;
% SYNCFMT.IMU.AccX = xacc;
% SYNCFMT.ATT(1).Pitch = pitch;
% SYNCFMT.ATT(1).Roll = roll;
% SYNCFMT.XKF1(1).VD = -roc;


g = 9.807;
lp = 5;




% if ~exist('Time')
% Time = cumtrapz((1/syncFreq).*ones(size(SYNCFMT.RPM.rpm1,1),1)) ;
% end

% SCALED_PRESSURE2 = latestmsgs(MESSAGE.SCALED_PRESSURE2,1);
% temperature = double(SCALED_PRESSURE2.Payload.temperature_press_diff)./100;
% pressure = double(SCALED_PRESSURE.Payload.press_abs) .* 100;
%
density = pressure ./ ((temperature+273.15).*287.05);
airspeed=  airspeed ./ sqrt(density./1.225) ;
% AOA =2.5;
% POWER AVAIL
[powavail,T] = fcn_poweravail(rpm,propd,density,airspeed,AOA,vehicle,prop);



% xacc = smooth(xacc,1000);
[D] = fcn_drag(m,zacc,xacc,T,AOA,density,airspeed,vehicle);

vdiff = diff(airspeed)./seconds(diff(Time));
vdiff = [0; vdiff];
accelpow = m.* vdiff .* airspeed;
accelpow(isnan(accelpow)) = 0;
% accelpow = lowpass(accelpow,(0.5),10);
accelpow = lowpass(accelpow,(0.1),10);
accelpow = smooth(accelpow,'moving',10000);
accelpow2 = fcn_accelpower(m,xacc,zacc,g,pitch,roll,airspeed,AOA);
% accelpow = accelpow-nanmean(accelpow);

accelpow2(isnan(accelpow2)) = 0;
accelpow2 = highpass(accelpow2,(0.001),10);

% lp = (lp*0.995) + (accelpow*0.005);
% accelpow = accelpow-lp;

roc2 = diff(alt)./seconds(diff(Time));
roc2 = [0; roc2];

% figure(3)
% clf(3)
% grid on
% hold on
% plot(Time,roc)
% plot(Time,roc2)
% 

roc3 = airspeed.*(sind(pitch).*cosd(AOA)) -airspeed.*(cosd(pitch).*cosd(roll).*sind(AOA));
% 
% plot(Time,roc3)
% legend('roc','roc2','calc')

climbpowreq = m .* g .* roc;
dragpowreq = D .* airspeed;

% CALCULATE POWER REQUIRED

powreq = accelpow + climbpowreq + dragpowreq;

P_thermal = powreq - powavail;

V_thermal = P_thermal./ (m*g); %thermal speed
% V_thermal2 = roc - roc3

% plot(Time,roll./20)
%
figure(2);
clf(2);
s(1)=subplot(4,1,1);
plot(Time,powavail,'.');
grid on
%             ylim([-30 100])
ylabel('powavail')
s(2)=subplot(4,1,2);
hold on
plot(Time,accelpow,'.r');
plot(Time,accelpow2,'.k');

% plot(Time,kineticHP,'.');
plot(Time,climbpowreq,'.b');
grid on
ylabel('accel/climb')
s(3) = subplot(4,1,3);
plot(Time,dragpowreq,'.');
grid on
ylabel('drag')
ylim([10 40])
s(4) = subplot(4,1,4);
hold on
plot(Time,V_thermal,'.');
% plot(Time,V_thermal2,'.r');
cleaned = 0;
% for i = 1:length(V_thermal)
%     if isnan(V_thermal(i))
%         cleaned = cleaned;
%     else
%     cleaned = cleaned*0.6 + V_thermal(i)*0.4;
%     end
%     V_clean(i) = cleaned;
% end
% plot(Time,V_clean,'.');
ylabel('V thermal')
grid on
ylim([-3 3])

linkaxes(s,'x')
%     xlim([7300 8800])

