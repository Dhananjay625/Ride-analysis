clc;
%% constants
g=9.81;    % gravity [m/s^2]
L = 2.64;                     % wheelbase (m)
%% Vehicle Parameters 
M_initial = 1385;                    % base vehicle mass (kg)

front = 0.6;            % 60% front
rear  = 0.4;            % 40% rear
mp = 79;                      % passenger mass (kg)
xp = 0.2;                     % passenger position from front axle (m)

ml = 10;                      % luggage mass (kg)
xl = 2.3;                     % luggage position from front axle (m)

W = M_initial * g;                                  % base vehicle weight (N)
Wf_initial = front * W;                             % front axle load (N)
Wr_initial = rear  * W;                             % rear axle load (N)

a_initial = (Wr_initial * L) / W;                   % distance from front axle to CG (m)
b_initial = L - a_initial;                          % distance from CG to rear axle (m)

M = M_initial + mp + ml;                            % total vehicle mass (kg)
W = M * g;                                          % total vehicle weight (N)

% New CG position after adding payload
a = (M_initial*a_initial + mp*xp + ml*xl) / M;      % front axle to CG (m)
b = L - a;                                          % CG to rear axle (m)

% New axle loads
Wf = W * b / L;               % front axle load (N)
Wr = W * a / L;               % rear axle load (N)


mu_total = 0.1 * M;             % total unsprung mass (kg)
musf = mu_total / 2;            % front unsprung mass (kg)
musr = mu_total / 2;            % rear unsprung mass (kg)
ms = M-musf-musr;               % Sprung mass [kg] 
msf = ms*(b/L);                 % front sprung mass (kg)
msr = ms*(a/L);                 % rear sprung mass (kg)


ff=1.2;                        % front ride frequency [Hz]
fr=1.2*ff;                      % Rear ride frequency [Hz]
I_initial = 1728.758;           % pitch inertia (kg.m^2)

I = I_initial + mp*(xp - a)^2 + ml*(xl - a)^2;



%% Suspension paramters


kfspring = (2*pi*ff)^2*(msf);      % Front Spring stiffness [N/m]
krspring = ((2*pi*fr)^2)*(msr);      % Rear Spring stiffness [N/m]

Fof = ((msf)*g);             %Front Spring Preload [N]
For = ((msr)*g);             %Rear Spring Preload [N]

zf = 0.35;                           %front damping ratio []
zr = 0.4;                           %Rear damping ratio []

ccf = 2*sqrt(kfspring*msf);           %Front crtical damping [Ns/m]
ccr = 2*sqrt(krspring*msr);           %Rear crtical damping [Ns/m]

cfdamper = zf*ccf;                   %Front Damping coefficient [Ns/m]
crdamper = zr*ccr;                   %Rear Damping coefficient [Ns/m]

ktyre = 290000;       %Tyre stiffness [N/m]

zOftyre = (((msf)+musf)*g/ktyre); %Deflection of the front tyre at the equilbrium [m]
zOrtyre = (((msr)+musr)*g/ktyre); %Deflection of the rear tyre at the equilbrium [m]
%% Input values
v = 4;       % vehicle speed (m/s)

h = 0.03;      % bump height 
l = 2;         % bump length 

T  = l / v;   % bump duration
time = L/v;
t0 = 0.5;     % front wheel bump start time
t1 = t0 + time;% rear wheel bump start time

%% simulation run for half sinebump
load_system("DOF4_sinebump");
out = sim("DOF4_sinebump");

%% Approximate front and rear wheel hop frequencies
ridefreq_f=sqrt(kfspring/msf)/(2*pi);
ridefreq_r=sqrt(krspring/msr)/(2*pi);
fhf=sqrt((ktyre+kfspring)/musf)/(2*pi);
fhr=sqrt((ktyre+krspring)/musr)/(2*pi);
ridefreq_f
ridefreq_r
fhf
fhr

rms_sprung=rms(out.zsdotdot.data)
% rms_unsprungf=rms(out.zusfdotdot.data)
% rms_unsprungr=rms(out.zusrdotdot.data

%% Road input front
 figure;
plot(out.zrf.time,out.zrf.data,'Linewidth',1.5);
hold on;
plot(out.zrr.time,out.zrr.data,'Linewidth',1.5);
xlabel('Time [s]');
ylabel('Road Input Front [m]');
legend('Front','Rear')
grid on
xlim([0,5]);
title('Road Input vs Time');

%% masses vertical Displacement
figure;
xlabel('Time [s]');
ylabel('Sprung Mass Vertical Displacement [m]');
xlim([0 5]);
grid on
title('Mass Vertical Displacement vs Time');
hold on
plot(out.zs.time,out.zs.data,'Linewidth',1.5);
plot(out.zusf.time,out.zusf.data,'Linewidth',1.5);
plot(out.zusr.time,out.zusr.data,'Linewidth',1.5);
plot(out.tout,out.zrf.data,'Linewidth',1.5);
plot(out.tout,out.zrr.data,'Linewidth',1.5);

legend('Sprung','Front Unsprung','Rear Unsprung','Front Road Input','Rear Road Input');
hold off


%% mass vertical velocity
figure;
plot(out.zsdot.time,out.zsdot.data,'Linewidth',1.5);
hold on
plot(out.zusfdot.time,out.zusfdot.data,'Linewidth',1.5);
plot(out.zusrdot.time,out.zusrdot.data,'Linewidth',1.5);
xlabel('Time [s]');
ylabel('Mass Vertical velocity [m/s]');
grid on
title('Masses Vertical velocity vs Time');
legend('Sprung','Front Unsprung','Rear Unsprung');
hold off;
xlim([0 5]);
%% mass vertical acc
figure;
plot(out.zsdotdot.time,out.zsdotdot.data,'Linewidth',1.5);
hold on
plot(out.zusfdotdot.time,out.zusfdotdot.data,'Linewidth',1.5);
plot(out.zusrdotdot.time,out.zusrdotdot.data,'Linewidth',1.5);
xlabel('Time [s]');
ylabel('Mass Vertical acc [m/s^2]');
grid on
title('Masses Vertical Acc vs Time');
legend('Sprung','Front Unsprung','Rear Unsprung');
hold off;
xlim([0 5]);
%% simulation run for frequency analysis
load_system("DOF4_frequency_input.slx")
%open_system("DOF4_frequency_input.slx")
out = sim("DOF4_frequency_input.slx");
%% Sprung mass vertical acc
dt = 0.001;
figure;
[amps,phases, freqs] = fft_V(out.zsdotdot.data, dt);
plot(freqs, amps, 'LineWidth', 1.5);
hold on;
low_idx = freqs > 0.5 & freqs < 5;
[pks_low, locs_low] = findpeaks(amps(low_idx), freqs(low_idx));
[pk1, i1] = max(pks_low);
freq1 = locs_low(i1);
high_idx = freqs > 8 & freqs < 20;
[pks_high, locs_high] = findpeaks(amps(high_idx), freqs(high_idx));
[pk2, i2] = max(pks_high);
freq2 = locs_high(i2);
plot(freq1, pk1, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot(freq2, pk2, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(freq1, pk1, sprintf('  %.2f Hz', freq1));
text(freq2, pk2, sprintf('  %.2f Hz', freq2));
xlabel('Frequency [Hz]');
ylabel('Sprung Mass Vertical Acceleration [m/s^2]');
title('Sprung Mass Vertical Acceleration vs Frequency');
grid on;
xlim([0 30]);
%% UnSprung mass vertical acc
figure;
[ampusf,phaseusf,frequsf]=fft_V(out.zusfdotdot.data,dt);
[ampusr,phaseusr,frequsr]=fft_V(out.zusrdotdot.data,dt);
plot(frequsf,ampusf,'Linewidth',1.5);
xlabel('Frequency [Hz]');
ylabel('UnSprung Mass Vertical Acceleration [m/s^2]');
xlim([0 30]);
grid on
title('UnSprung Mass Vertical Acceleration vs Frequency');
hold on
plot(frequsr,ampusr,'Linewidth',1.5);
legend('Front','Rear');
hold off
%% Sprung mass pitch acc
dt = 0.001;
figure;
[ampsp,phasesp, freqsp] = fft_V(out.pitchdotdot.data, dt);
plot(freqsp, ampsp, 'LineWidth', 1.5);
hold on;
low_idx = freqsp > 0.5 & freqsp < 5;
[pks_low, locs_low] = findpeaks(ampsp(low_idx), freqsp(low_idx));
[pk1, i1] = max(pks_low);
freq1 = locs_low(i1);
high_idx = freqsp > 8 & freqsp < 20;
[pks_high, locs_high] = findpeaks(ampsp(high_idx), freqsp(high_idx));
[pk2, i2] = max(pks_high);
freq2 = locs_high(i2);
plot(freq1, pk1, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot(freq2, pk2, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(freq1, pk1, sprintf('  %.2f Hz', freq1));
text(freq2, pk2, sprintf('  %.2f Hz', freq2));
xlabel('Frequency [Hz]');
ylabel('Sprung Mass Vertical Acceleration [m/s^2]');
title('Sprung Mass Vertical Acceleration vs Frequency');
grid on;
xlim([0 30]);

