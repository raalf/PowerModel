function audiovario(thermal)

 lift = [0.3 4 ; 400 1800 ; 1 0.2];
sink = [-4 -0.3 ; 200 300 ];


while 1<2
 % if toc(variotimer) > variowait

      
        
        if thermal > 5
            thermal = 5;
        elseif thermal < -5
            thermal = -5
        end
        try
            if thermal > 0
                duration = interp1(lift(1,:),lift(3,:),thermal);
                if duration<0.1
                    duration = 0.1;
                end
                w = 2*pi*interp1(lift(1,:),lift(2,:),thermal); % Radian Value To Create Tone

                Fs = 14400;                                     % Sampling Frequency
                t  = linspace(0, duration, Fs.*duration);                        % One Second Time Vector
                s = sin(w*t) ;                                   % Create Tone
                s(end-1000:end) = 0;
                sound([s], Fs)                                    % Produce Tone As Sound
                variowait = (duration)

            elseif thermal <0
                w = 2*pi*interp1(sink(1,:),sink(2,:),thermal);   % Radian Value To Create 1kHz Tone

                Fs = 14400;                                     % Sampling Frequency
                duration = 0.5;
                t  = linspace(0, duration, Fs.*duration);                        % One Second Time Vector
                s = sin(w*t);                                   % Create Tone
                sound([s], Fs)                                    % Produce Tone As Sound
                variowait = (0.5-0.01);

            end
        end
        variotimer = tic;
    % end


end


%%
Fs = 14400;                                     % Sampling Frequency
t  = linspace(0, 1, Fs);                        % One Second Time Vector
w = 2*pi*1000;                                  % Radian Value To Create 1kHz Tone
s = sin(w*t);                                   % Create Tone
player = audioplayer(s, Fs)                                    % Produce Tone As Sound
player.play
