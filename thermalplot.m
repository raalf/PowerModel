
%% Offline Thermal Plot
% added zacc geoplot

% Things to address:
% Roll angle not accounted for
% Data loss/rates change

clc
clear

density = 1.2; % Check
S = 0.378;
m = 1.8; % Confirm mass: 1.81 + battery?
g = 9.807;

% Cd0 = 0.02423997;
% Cdi = 0.03166895;

Cd0 = 0.018;
Cdi = 0.0516;

%% Initialize Plot

% Check for internet
url = 'https://google.com';

% Try to connect to it.
[~,internet] = urlread(url);

plottimer = tic;

buffsize = 90; %number of points to show
freq = 30; %frequency to average at Hz (need to be high to get solar data)
plotper = 1; %plot period in s

latbuff = NaN(1,buffsize);
longbuff = NaN(1,buffsize);
thermbuff = NaN(1, buffsize);
thermbuffsmoothed = NaN(1, buffsize);
zaccbuff = NaN(1, buffsize);
zaccbuffsmoothed = NaN(1, buffsize);
pybuffavail = NaN(2,buffsize);
pybuffreq = NaN(2,buffsize);
pybuffbatt = NaN(2, buffsize);

count = 0;
xacc = 0;
roc = 0;
P_thermal = 0;
V_thermal_smoothed = 0;
zacc_smoothed = 0;
lp = 5;
powavail = 0;
new_voltage = 0;
avgpoweravail = 0;
avgpowerreq = 0;
avgpowerbatt = 0;

CLmindrag = 0.000000;
Cd0 = 0.02477653;
e = 0.36179696;
AR = 17.05;
S = 0.378460;
m = 1.793;
rho = 1.204;
g = 9.81;
b = linspace(0,0.35,50);
V = 12.3;
propd = 0.2794; %0.4699

figure(66);
clf(66);   
gx(4)=geoaxes;
gx(4).ZoomLevel = 17;
colormap(gx(4),(cool));

cb4 = colorbar(gx(4)); %,'Position',[.88 .11 .04 .815]);
cb4.Label.FontWeight = 'Bold';
cb4.Label.FontSize = 10;
cb4.FontWeight = 'Bold';
cb4.FontSize = 10;
cb4.Label.String = 'Thermal Strength (m/s)';

gx(4).LatitudeAxis.TickValues=[];
gx(4).LongitudeAxis.TickValues=[];

gx(4).LatitudeAxis.Label.String='';
gx(4).LongitudeAxis.Label.String='';

title('thermal')

figure(69);
clf(69);
gx2(4)=geoaxes;
gx2(4).ZoomLevel = 17;
colormap(gx2(4),(cool));

cb5 = colorbar(gx2(4)); %,'Position',[.88 .11 .04 .815]);
cb5.Label.FontWeight = 'Bold';
cb5.Label.FontSize = 10;
cb5.FontWeight = 'Bold';
cb5.FontSize = 10;
cb5.Label.String = 'Zacc (m/s^2)';

gx2(4).LatitudeAxis.TickValues=[];
gx2(4).LongitudeAxis.TickValues=[];

gx2(4).LatitudeAxis.Label.String='';
gx2(4).LongitudeAxis.Label.String='';

title('zacc')

if internet == true
% use regular satellite map
geobasemap(gx(4),'satellite');
geobasemap(gx2(4),'satellite');

else
    % offline map stuff
    % Connect to Local Server
    directory = 'C:\ProgramData\Mission Planner\gmapcache\TileDBv3\en\GoogleSatelliteMap';
    % Try to open a connection to the server
    try
        % try to contact the local server for 2 seconds, see if it responds
        response = webread('http://localhost:8000', weboptions('Timeout', 2));
    
        % If the request is successful, print
        disp('Http server is already running');
    
    catch
        % If the request fails, start the server
        disp('Starting server...');
        system(['start python -m http.server 8000 ', ' --directory "', directory, '"'], '-echo');
    end
    
    % create offline basemap
    basemapName = "MissionPlannerCache";
    URL = 'http://localhost:8000/{z}/{y}/{x}.jpg';
    addCustomBasemap(basemapName, URL, "Attribution", '')
    geobasemap(gx(4),basemapName);
    geobasemap(gx2(4),basemapName);
end


% thermal strength plot
figure(68);
clf(68);
timeBuffer = [];
pavailBuffer = [];
preqBuffer = [];
powavail_plot = plot(nan, nan);
hold on;
powreq_plot = plot(nan, nan);
xlabel('Time (ms)');
ylabel('Power (W)');
title('Time vs Power avail/req');
legend('Power Available', 'Power Required');
grid on;

%% Connect to Mavlink
% UDP Client to connect
dialect = mavlinkdialect("ardupilotmega.xml"); %this might need to include the special AP.xml as well
gcsNode = mavlinkio(dialect);
gcsPort = 14562; %whatever port. I am using mavproxy right now to mirror the mavlink stream
connect(gcsNode,"UDP", 'LocalPort', gcsPort);

listClients(gcsNode)
listConnections(gcsNode)
listTopics(gcsNode) %this will list all the available message topics on the stream

uavClient = mavlinkclient(gcsNode,1,1); %this needs to be the sysid and comid of the UAV

MESSAGE.GLOBAL_POSITION_INT=mavlinksub(gcsNode,uavClient,'GLOBAL_POSITION_INT');
MESSAGE.SYSTEM_TIME=mavlinksub(gcsNode,uavClient,'SYSTEM_TIME');
MESSAGE.BATTERY_STATUS=mavlinksub(gcsNode,uavClient,'BATTERY_STATUS');
MESSAGE.RAW_IMU=mavlinksub(gcsNode, uavClient, 'RAW_IMU');
MESSAGE.VFR_HUD=mavlinksub(gcsNode, uavClient, 'VFR_HUD');
MESSAGE.LOCALPOSITION_NED=mavlinksub(gcsNode, uavClient, 'LOCAL_POSITION_NED');
MESSAGE.SCALED_PRESSURE=mavlinksub(gcsNode,uavClient,'SCALED_PRESSURE');
MESSAGE.RPM=mavlinksub(gcsNode,uavClient,'RPM');
MESSAGE.ATTITUDE=mavlinksub(gcsNode,uavClient,'ATTITUDE');
pause(1)




%% Update Plot

while 1<2
    timer = tic;

    while toc(timer) < (1/freq)

        GLOBAL_POSITION_INT=latestmsgs(MESSAGE.GLOBAL_POSITION_INT,1);

        SYSTEM_TIME=latestmsgs(MESSAGE.SYSTEM_TIME,1);
        time = SYSTEM_TIME.Payload.time_boot_ms;
        unixTime = SYSTEM_TIME.Payload.time_unix_usec;

        RAW_IMU = latestmsgs(MESSAGE.RAW_IMU,1);
        VFR_HUD = latestmsgs(MESSAGE.VFR_HUD,1);
        LOCALPOSITION_NED =  latestmsgs(MESSAGE.LOCALPOSITION_NED,1);
        SCALED_PRESSURE = latestmsgs(MESSAGE.SCALED_PRESSURE, 1);
        RPMraw = latestmsgs(MESSAGE.RPM, 1);
        ATTITUDE = latestmsgs(MESSAGE.ATTITUDE,1);
        BATTERY_STATUS=latestmsgs(MESSAGE.BATTERY_STATUS,1);
        temperature = 20;
        pressure = double(SCALED_PRESSURE.Payload.press_abs) .* 100;

        density = pressure ./ ((temperature+273.15).*287.05);

        % Getting airspeed
        airspeed = double(VFR_HUD.Payload.airspeed);

        % Getting descent rate
        roc = (-double(LOCALPOSITION_NED.Payload.vz));

        % Getting ESC RPM
        rpm = double(RPMraw.Payload.rpm1);
        n = rpm ./ 60;
        if rpm<3500
            n=0;
        end

        xacc = double(RAW_IMU.Payload.xacc)./ 100;
        zacc = -double(RAW_IMU.Payload.zacc) ./ 100;

        pitch = double(ATTITUDE.Payload.pitch) .*180./pi;
        yaw = (double(ATTITUDE.Payload.yaw)) ;
        current = double(BATTERY_STATUS.Payload.current_battery)/100;
        voltage = double(BATTERY_STATUS.Payload.voltages(1))/1000;
        powerbatt = current*voltage;
    end

       
    if toc(plottimer) > plotper

        % Average all data we've grabbed since the last plot
        lat = double(GLOBAL_POSITION_INT.Payload.lat)./10000000;
        latbuff = [latbuff(2:end) lat];
        long = double(GLOBAL_POSITION_INT.Payload.lon)./10000000;
        longbuff = [longbuff(2:end) long];
        
        % CALCULATING DRAG

        CL = 2*m*zacc./(S*rho*(V.^2));
        Cd = Cd0 + (CL - CLmindrag).^2./(pi*e*AR);
        D = Cd*S*rho.*(V.^2)./2;

        % Advance ratio
        J = airspeed ./ (n .* propd);
    
        % CALCULATING CT

        x = J;
        y = rpm;
        p00 =     0.05945;
        p10 =    -0.01384;
        p01 =   1.239e-05;
        p20 =     -0.1057;
        p11 =  -3.204e-06;
        p02 =  -9.234e-10;

        ct = p00 + p10.*x + p01.*y + p20.*x.^2 + p11.*x.*y + p02.*y.^2;
        
        if rpm<3500
            ct=0;
        end
        
        powavail = (ct*n*n*(propd)^4*density ) * airspeed;

        % AVERAGE POWER AVAILABLE
        
        pybuffavail = [pybuffavail(:,2:end) [yaw ; powavail]];
        try
        %continous average over one loiter
        f=(rad2deg(wrapTo2Pi(unwrap((pybuffavail(1,:)+pi))-yaw)))-180;
        zerocross = find(f(2:end).*f(1:end-1)<0)  ;
        idx_last_loiter = (zerocross(end-1));
        avgpoweravail = mean(pybuffavail(2,zerocross(end-1):end), 'omitnan');
        catch
        end

        % AVERAGE BATTERY POWER
        
        pybuffbatt = [pybuffbatt(:,2:end) [yaw ; powerbatt]];
        try
        %continous average over one loiter
        fbatt=(rad2deg(wrapTo2Pi(unwrap((pybuffbatt(1,:)+pi))-yaw)))-180;
        zerocrossbatt = find(fbatt(2:end).*fbatt(1:end-1)<0)  ;
        idx_last_loiter_batt = (zerocrossbatt(end-1));
        
        avgpowerbatt = mean(pybuffbatt(2,zerocrossbatt(end-1):end), 'omitnan');
        catch
        end
        
        % CALCULATE POWER REQUIRED 
        
        accelpowreq = m * (xacc-g*sind(pitch)) * airspeed;
        lp = (lp*0.995) + (accelpowreq*0.005);
        accelpowreq = accelpowreq-lp;
        climbpowreq = m * g * roc;
        dragpowreq = D * airspeed;
        powreq = accelpowreq + climbpowreq + dragpowreq;
        
        % AVERAGE POWER REQUIRED
        
        pybuffreq = [pybuffreq(:,2:end) [yaw ; powreq]];
        try
        %continous average over one loiter
        frequired=(rad2deg(wrapTo2Pi(unwrap((pybuffreq(1,:)+pi))-yaw)))-180;
        zerocrossreq = find(frequired(2:end).*frequired(1:end-1)<0)  ;
        idx_last_loiter_req = (zerocrossreq(end-1));
        
        avgpowerreq = mean(pybuffreq(2,zerocrossreq(end-1):end), 'omitnan');
        catch
        end

        % CALCULATE THERMAL 
        
        P_thermal = powreq - powavail;

        V_thermal = P_thermal./ (m*g); %thermal speed
        V_thermal_smoothed = V_thermal_smoothed*0.4 + V_thermal*0.6;

        thermbuff = [thermbuff(2:end) V_thermal];
        thermbuffsmoothed = [thermbuffsmoothed(2:end) V_thermal_smoothed];

        % PLOT 

        figure(66)

        try
            delete(findobj(gx(4),'type','Scatter'))
        catch
        end

        thermbuffplot = thermbuffsmoothed;

        try
            mint = min(thermbuffplot(2:end-1));
            maxt = max(thermbuffplot(2:end-1));
            if abs(mint)>abs(maxt)
                caxis(gx(4),[mint abs(mint)])
            else
                caxis(gx(4),[-(maxt) maxt])
            end
            % % caxis(gx(4),[mint maxt])
        catch
            caxis(gx(4),'auto')
        end

        sz=( (thermbuffplot(2:end-1) + (-mint))./ (maxt-mint) .*40 + 25);
        hold on
        h=geoscatter(gx(4),latbuff(2:end-1),longbuff(2:end-1),sz,thermbuffplot(2:end-1),...
            'MarkerFaceColor','flat');

        geoscatter(gx(4),lat,long,50,V_thermal,'filled','MarkerEdgeColor','w','LineWidth', 2)
        gx(4).ZoomLevelMode='auto';

        if gx(4).ZoomLevel > 17
            gx(4).ZoomLevel = 17;
        end
        currentAxis = gca;
        currentAxis.Position(3) = 0.5;
        
        % Average power box
        delete(findall(gcf,'Tag','avgpowerprop'));
        str = {["AVG POWER AVAIL: " + avgpoweravail + " W"], ["AVG POWER REQUIRED: " + avgpowerreq + " W"], ["AVG POWER BATT: " + avgpowerbatt + " W"]};
        annotation('textbox', [0.25, 0.005, 0.1, 0.1], 'String', str, 'tag', 'avgpowerprop')
        drawnow;

        titlecolor=interp1(linspace(gx(4).CLim(1),gx(4).CLim(2),length(gx(4).Colormap)), gx(4).Colormap,V_thermal,'nearest','extrap');
        str = "Thermal: " + round(V_thermal,1) + " m/s ";

        title(str,'BackgroundColor',titlecolor)

        dateTime = datestr(datetime(unixTime/1e6, 'ConvertFrom', 'posixtime', 'TimeZone', 'America/Toronto'), 'dd-mmm-yyyy HH:MM:SS');
        subtitle(dateTime + "  microsec: " + time*1000+ "  Bin xx");
        
        figure(69)

        zacc_smoothed = zacc_smoothed*0.4 + zacc*0.6;

        zaccbuff = [zaccbuff(2:end) zacc];
        zaccbuffsmoothed = [zaccbuffsmoothed(2:end) zacc_smoothed];
        try
            delete(findobj(gx2(4),'type','Scatter'))
        end

        zaccbuffplot = zaccbuffsmoothed;

        try
            mint = min(zaccbuffplot(2:end-1));
            maxt = max(zaccbuffplot(2:end-1));
            if abs(mint)>abs(maxt)
                caxis(gx2(4),[6 abs(mint)])
            else
                caxis(gx2(4),[6 maxt])
            end
            % % caxis(gx(4),[mint maxt])
        catch
            caxis(gx2(4),'auto')
        end

        sz=( (zaccbuffplot(2:end-1) + (-mint))./ (maxt-mint) .*40 + 25);
        hold on
        h=geoscatter(gx2(4),latbuff(2:end-1),longbuff(2:end-1),sz,zaccbuffplot(2:end-1),...
            'MarkerFaceColor','flat');

        geoscatter(gx2(4),lat,long,50,zacc,'filled','MarkerEdgeColor','w','LineWidth', 2)
        gx2(4).ZoomLevelMode='auto';

        if gx2(4).ZoomLevel > 17
            gx2(4).ZoomLevel = 17;
        end

        titlecolor=interp1(linspace(gx2(4).CLim(1),gx2(4).CLim(2),length(gx2(4).Colormap)), gx2(4).Colormap,zacc,'nearest','extrap');
        str = "Zacc: " + round(zacc,2) + " m/s^2 ";

        title(str,'BackgroundColor',titlecolor)

        dateTime = datestr(datetime(unixTime/1e6, 'ConvertFrom', 'posixtime', 'TimeZone', 'America/Toronto'), 'dd-mmm-yyyy HH:MM:SS');
        subtitle(dateTime + "  microsec: " + time*1000 + "  Bin xx");
        
        % Update buffers
        timeBuffer = [timeBuffer time];
        pavailBuffer = [pavailBuffer powavail];
        preqBuffer = [preqBuffer powreq];


        % Limit buffer size
        if length(timeBuffer) > 1000
            timeBuffer = timeBuffer(end-999:end);
            powavailBuffer = powavailBuffer(end-999:end);
            powreqBuffer = powreqBuffer(end-999:end);
        end

        % Update the plots
        set(powavail_plot, 'XData', timeBuffer, 'YData', pavailBuffer);
        set(powreq_plot, 'XData', timeBuffer, 'YData', preqBuffer);
        drawnow;
        
        plottimer = tic;
        count = 0;
        solarcount = 0;
        escpow =0;
        VE_ppv_W=0;
        acvoltage =0;
        voltcount = 0;
        P_thermal = 0;
    end
end

%% Disconnect - Is this needed?

disconnect(gcsNode,'Localhost'); %doesn't always to work
clear all
close all
clc
clear

%%
% Needs better function with n, torque and velocity
function [eff] = eta_sys(J)
eff = (-10.75 .* J.^4) + (4.29 .* J.^3) - (0.6074 .* J.^2) + (1.65 .* J) + 9.01e-05;
end