clc;
clear all;

%% Loading Simulator Data
load('Endurance17_Aero.mat')
data = csvread('50Amptest2_two.csv');
time2 = data(:,1);
energus_Temp = data(:,7);
current = data(:,3);
terminal_temp = data(:,6)./100;
amb1_temp = data(:,5)./100; % ambient temperature below cardboard

%% Variables
Ti = 32; % deg celsius
Tinf = 42; % deg celsius
m = 22*10^-3; % mass of the copper in kg
c = 385; %; %W/m-K ??? should be J/kg-K 385 -> copper; 1007 -> air
h_bar = 4; % guess, W/m-K turns out 4 is quite accurate instead of 15
%So basically: 4 is what I found to be a reasonable value
% 4 must be a good estimate because the flow rate is so small!
As = 0.791*0.838*0.0254^2*2; % surface area of the busbar; two times the Surface Area

%% Loop to Calculate Correct Endpoint of Heat Up in Data
datastart = 0;
dataend = 0;
jj=1; % looping variable
Ti_new = 0;
% **Need to add to this loop: finding qmax and qstart
while jj <= length(time2);%length(time)/100
    %disp(datastart); disp(current(jj));
    if current(jj) > 0 && datastart == 0
        startpoint = jj;
        datastart = datastart + 1;
    end
    if current (jj) == 0 && datastart == 1 && dataend == 0
        %disp('on');
        endpoint = jj;
        dataend = 1;
    end
    %qconv_jj(jj) = h_bar*As*(terminal_temp(jj)-amb1_temp(jj));
    jj = jj + 1;
end

Tsurface = terminal_temp(endpoint); % T in C at surface sensor when it appears to hit a steady q this is at about 900 seconds
Tamb = amb1_temp(endpoint); % T in C at amb sensor when it appears to hit a steady q
qconv = h_bar*As*(Tsurface-Tamb);
qbat = 0.4; % unit in W
t=0:1:7000;
Tss = qbat/(h_bar*As)+Tinf; % deg celsius
tt = 2*m*c/(h_bar*As); % time constant; two times the masses

%% 30 Lap Loop
vector = AccelDeccelValues();
i = 0; % Iteration Var
j = 0; % accel counter
k = 0; % deccel counter
tempcounter = 0; % counter to iterate values into the Temp vector
otimecounter = 0; % overall time counter in seconds
timevector = []; % time vector, useful for plotting; increments the otimecounter values
laps = 0; % laps that the car will run
while laps <= 30 % run the code for 30 laps
    while i < length(vector)
        %fprintf('current i value is: %2.0f\n', i);
        %fprintf('current vector value is: %2.0f\n', vector(i+1));
        timevector(tempcounter+1) = otimecounter;
        %disp(timevector(tempcounter+1))
        if i > 0
                Ti_new = Temp(tempcounter+1-1);
                timenewstart = timevector(tempcounter+1-1);
        end
        %fprintf('current Ti_new value is: %2.4f\n', Ti_new);
        %fprintf('current tempcounter value is: %2.0f\n', tempcounter);
        %fprintf('current timenewstart value is: %2.7f\n', timenewstart); 
        if vector(i+1) > 0% If we are accelerating, run this code
            %disp('accel');
            t_on_accel = abs(vector(i+1));
            % Recall Ti_new has been defined initially so that the "initial
            % value does not change as the loop iterates
            timecounter = 0;
            while j < t_on_accel

                %fprintf('current j value is: %2.0f\n', j);
                if i == 0 && laps == 0
                    Temp(tempcounter+1) = (Ti - Tss)*exp(-timecounter/tt)+Tss;
                else
                    Temp(tempcounter+1) = (Ti_new - Tss)*exp((-timecounter)/tt)+Tss;
                end
                timevector(tempcounter+1) = otimecounter;
                tempcounter = tempcounter + 1;
                otimecounter = otimecounter + 0.00105413; % overall scope time counter
                timecounter = timecounter + 0.00105413; % short scope time counter - just for the loop

                j = j + 1;
            end
            j = 0;
            %fprintf('Temp is: %2.2f, and the time on accel is: %2.2f.\n', Temp(i+1), t_on_accel);

            if i == length(vector)-1%length(outputs.vs(1).time) % if we are at the end, then the 
                % code is finished.
                break
            else
                if vector(i+2) < 0.01 %if the next acceleration term is less than zero %t_on_accel == t_on_accel_max % once the accelerating time reaches 4 seconds
                    % then we assume that the vehicle is deccelerating
                    %t_accel_active = 0;
                    %t_deccel_active = 1;
                    t_on_accel = 0;
                end

            end


        else %if vector(i+1) < 0.01 % t_on_deccel == 1 (active) if we are deccelerating, run this code
            %disp('deccel');
            t_on_deccel = abs(vector(i+1));
            % Recall Ti_new has been defined initially        
            timecounter = 0;
            while k < t_on_deccel
                %fprintf('current k value is: %2.0f\n', k);
                if i == 0 && laps == 0
                    Temp(tempcounter+1) = (Ti - Tinf)*exp(-(h_bar*As)/(m*c)*timecounter) + Tinf;
                else
                    Temp(tempcounter+1) = (Ti_new-Tinf)*exp(-(h_bar*As)/(m*c)*(timecounter)) + Tinf;
                end
                timevector(tempcounter+1) = otimecounter;
                tempcounter = tempcounter + 1;
                otimecounter = otimecounter + 0.00105413;
                timecounter = timecounter + 0.00105413;
                k = k + 1;
            end
            k = 0;
            %fprintf('Temp is: %2.2f, and the time on deccel is: %2.2f.\n', Temp(i+1), t_on_deccel);
            if i == 0
                 t_on_deccel = t_on_deccel + (outputs.vs(1).time(i+1));
            else
                t_on_deccel = t_on_deccel + (outputs.vs(1).time(i+1)-outputs.vs(1).time(i+1-1));
            end

            if i == length(vector)-1%length(outputs.vs(1).ax) % if we are at the end, then the 
                % code is finished.
                break % maybe need a more eloquent way to finish the code?
            else
                if vector(i+2) > 0.01 %t_on_deccel == t_on_deccel_max % once the accelerating time reaches 4 seconds
                    %t_deccel_active = 0;
                    %t_accel_active = 1;
                    t_on_deccel = 0;
                end

            end

        end

        i = i + 1;
    end
    i = 0; %reset i back to zero
    laps = laps + 1;
end
figure(1)
%i = 0:1:length(Temp)-1;
plot(timevector/60, Temp);
xlabel('Time (min)');
ylabel('Temperature (C)');
%figure(2)
timedep = 0:1:length(outputs.vs(1).time)-1;
%plot(timedep, outputs.vs(1).time);