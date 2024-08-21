
%% Offline Thermal Plot

% Things to address:
% Data loss/rates change

clc
clear

%% audio vario
audiovario =0;

if audiovario == 1

%[thermal strength; freq Hz; beep period]
 lift = [0.3 1 3 ; 550 700 2000 ; 1 1 0.2];
sink = [-3 -0.3 ; 200 255 ];
variotimer = tic;
variowait = 1;

end




%% Initialize Plot

% Check for internet
url = 'https://google.com';

% Try to connect to it.
[~,internet] = urlread(url);

plottimer = tic;
buffsize = 140; %number of points to show
freq = 60; %frequency to average at Hz (need to be high to get solar data)
plotper = 0.33; %plot period in s

latbuff = NaN(1,buffsize);
longbuff = NaN(1,buffsize);
thermbuff = NaN(1, buffsize);
thermbuffsmoothed = NaN(1, buffsize);
V_thermal_smoothed = 0;



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


if internet == true
    % use regular satellite map
    geobasemap(gx(4),'satellite');   
else
    % offline map stuff
    % Connect to Local Server
     
    % Try to open a connection to the server
    try
        % try to contact the local server for 2 seconds, see if it responds
        % response = webread('http://localhost:8000', weboptions('Timeout', 2));
response = webread('http://localhost:8000/',  weboptions('Timeout', 2));
        % If the request is successful, print
        disp('Http server is already running');

    catch
        % If the request fails, start the server
        
        if system('python --version')==9009
         disp('Trying to start server on WSL...');
         directory = '/mnt/c/ProgramData/Mission\ Planner/gmapcache/TileDBv3/en/GoogleSatelliteMap';
        system(['ubuntu run python -m http.server 8000 ', ' --directory "', directory, '"&' ]);  
        
        else 

        disp('Trying to start server on Windows...');
         directory = 'C:\ProgramData\Mission/Planner\gmapcache\TileDBv3\en\GoogleSatelliteMap';
  
        system(['start python -m http.server 8000 ', ' --directory "', directory, '"'], '-echo');
   
        end

    end

    % create offline basemap
    basemapName = "MissionPlannerCache";
    URL = 'http://localhost:8000/${z}/${y}/${x}.jpg';
    addCustomBasemap(basemapName, URL, "Attribution", '')
    geobasemap(gx(4),basemapName);
   
end


%% Connect to Mavlink
% UDP Client to connect
dialect = mavlinkdialect("ardupilotmega.xml"); %this might need to include the special AP.xml as well
gcsNode = mavlinkio(dialect);
gcsPort = 14562; %whatever port. I am using mavproxy right now to mirror the mavlink stream
connect(gcsNode,"UDP", 'LocalPort', gcsPort);

listClients(gcsNode)
listConnections(gcsNode)
tt=listTopics(gcsNode); %this will list all the available message topics on the stream

uavClient = mavlinkclient(gcsNode,1,1); %this needs to be the sysid and comid of the UAV

MESSAGE.GLOBAL_POSITION_INT=mavlinksub(gcsNode,uavClient,'GLOBAL_POSITION_INT');
MESSAGE.SYSTEM_TIME=mavlinksub(gcsNode,uavClient,'SYSTEM_TIME');
MESSAGE.NAMED_VALUE_FLOAT=mavlinksub(gcsNode,uavClient,'NAMED_VALUE_FLOAT');
pause(0.5)

NAMED_VALUE_FLOAT = latestmsgs(MESSAGE.NAMED_VALUE_FLOAT,1);
inittimer = tic
while strcmp(string(NAMED_VALUE_FLOAT.Payload.name(1:9)),"V_thermal") == 0
       NAMED_VALUE_FLOAT = latestmsgs(MESSAGE.NAMED_VALUE_FLOAT,1);   
       if toc(inittimer)==10
           error('No V_thermal from LUA')
       end
end
V_thermal_Lua = double(NAMED_VALUE_FLOAT.Payload.value);
%% Update Plot

while 1<2
    timer = tic;

    tt=listTopics(gcsNode); %this will list all the available message topics on the stream

    while toc(timer) < (1/freq)

        GLOBAL_POSITION_INT=latestmsgs(MESSAGE.GLOBAL_POSITION_INT,1);

        SYSTEM_TIME=latestmsgs(MESSAGE.SYSTEM_TIME,1);
        time = SYSTEM_TIME.Payload.time_boot_ms;
        unixTime = SYSTEM_TIME.Payload.time_unix_usec;

        NAMED_VALUE_FLOAT = latestmsgs(MESSAGE.NAMED_VALUE_FLOAT,1);
        if strcmp(string(NAMED_VALUE_FLOAT.Payload.name(1:9)),"V_thermal") == 1
            V_thermal_Lua = double(NAMED_VALUE_FLOAT.Payload.value)
        end



    end

  if audiovario == 1
 if toc(variotimer) > variowait

      thermal = thermbuff(end) ;
        
        if thermal > 3
            thermal = 3;
        elseif thermal < -3
            thermal = -3;
        end
        try
            if thermal > 0.3
                duration = interp1(lift(1,:),lift(3,:),thermal);
                if duration<0.2
                    duration = 0.2;
                end
                w = 2*pi*interp1(lift(1,:),lift(2,:),thermal); % Radian Value To Create Tone

                Fs = 14400;                                     % Sampling Frequency
                t  = linspace(0, duration, Fs.*duration);                        % One Second Time Vector
                w = w.*ones(size(t,2),1);

                fadet = min([8000,length(t)]);
                 if lastw == 2*pi*290
                    lastw =  2*pi*interp1(sink(1,:),sink(2,:),0.3);
                end
                w(1:fadet) = linspace(lastw,w(fadet),fadet); %blend from the last frequency

                s = sin(w'.*t) ;        %Create Tone             
                ramp = linspace(0, 1, 100);
                s(1:100) = s(1:100).*ramp; %fade in and out
                if thermal > 1
                    s(end-2000:end) = 0; %add pause
                    s(end-2099:end-2000) = s(end-2099:end-2000).*flip(ramp); %fade out
                else
                    s(end-99:end) = s(end-99:end).*flip(ramp);%fade out
                end
           
                sound([s], Fs)                                    % Produce Tone As Sound
                variowait = (duration-0.05);
                lastw = w(end-500);

            elseif thermal <-0.3
                w = 2*pi*interp1(sink(1,:),sink(2,:),thermal);   % Radian Value To Create 1kHz Tone
               
                Fs = 14400;                                     % Sampling Frequency
                duration = 1;
                t  = linspace(0, duration, Fs.*duration);                        % One Second Time Vector
                w = w.*ones(size(t,2),1);
                
                fadet = min([8000,length(t)]);
                if lastw == 2*pi*290
                    lastw =  2*pi*interp1(sink(1,:),sink(2,:),-0.3);
                end
                w(1:fadet) = linspace(lastw,w(fadet),fadet); %blend from the last frequency

                s = sin(w'.*t);                 
                ramp = linspace(0, 1, 200);
                s(1:200) = s(1:200).*ramp; %fade in and out
                s(end-199:end) = s(end-199:end).*flip(ramp);
                sound([s], Fs);                                    % Produce Tone As Sound
                variowait = (0.8); % need to get this time right
                lastw = w(end-200);
               toc(temptimer)
                temptimer = tic;
            else
                lastw = 2*pi*290;
                variowait=0.5;
            end
       

        end
        variotimer = tic;
 end

  end
    if toc(plottimer) > plotper

        % STORE LAT AND LONG IN BUFFERS
        lat = double(GLOBAL_POSITION_INT.Payload.lat)./10000000;
        latbuff = [latbuff(2:end) lat];
        long = double(GLOBAL_POSITION_INT.Payload.lon)./10000000;
        longbuff = [longbuff(2:end) long];

        % CALCULATE THERMAL
        V_thermal = V_thermal_Lua;
        
        V_thermal_smoothed = V_thermal_smoothed*0.4 + V_thermal*0.6;

        thermbuff = [thermbuff(2:end) V_thermal];
        thermbuffsmoothed = [thermbuffsmoothed(2:end) V_thermal_smoothed];

        % PLOT

        figure(66)

        try
            delete(findobj(gx(4),'type','Scatter'))
        catch
        end

        thermbuffplot = thermbuff;

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

        sz=( (thermbuffplot(2:end-1) + (-mint))./ (maxt-mint) .*70 + 25);
        hold on
        colors= abs(thermbuffplot(2:end-1));
        colors(colors>1) = 1;
        % colors(isnan(colors))=0;
        h=geoscatter(gx(4),latbuff(2:end-1),longbuff(2:end-1),sz,thermbuffplot(2:end-1),...
            'MarkerFaceColor','flat','MarkerFaceAlpha','flat','AlphaData',colors);

        geoscatter(gx(4),lat,long,50,V_thermal,'filled','MarkerEdgeColor','w','LineWidth', 2)
        gx(4).ZoomLevelMode='auto';

        if gx(4).ZoomLevel > 17
            gx(4).ZoomLevel = 17;
        end
        currentAxis = gca;
        currentAxis.Position(3) = 0.5;
           
        drawnow; 

        titlecolor=interp1(linspace(gx(4).CLim(1),gx(4).CLim(2),length(gx(4).Colormap)), gx(4).Colormap,V_thermal,'nearest','extrap');
        str = "Thermal: " + round(V_thermal,1) + " m/s " + "GPS Rate " + num2str(tt.MessageFrequency(tt.MessageName == "GLOBAL_POSITION_INT"));

        title(str,'BackgroundColor',titlecolor)
       
        plottimer = tic;

    end
end

%% Disconnect - Is this needed?

disconnect(gcsNode,'Localhost'); %doesn't always to work
clear all
close all
clc
clear



