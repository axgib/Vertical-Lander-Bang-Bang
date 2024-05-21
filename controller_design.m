%RR textbook: http://robotics.ucsd.edu/RR.pdf

% SSH Key: 192.168.7.2

close all
addpath(genpath('C:\Users\15103\MAE144_RR\RR'))

g = 9.81;% m/s^2

%Measurement of moment of inertia: https://www.youtube.com/watch?v=nwgd1CV__rs&t=342s
m = 1.315; % kg
d = 0.1;%distance from gimbal pivot to cg_tot TODO
strrad = 0.08255;% m
strlen = 0.55;% m
Txtot = 32.35/10; % oscilation period
Tytot = 29.3/10; % oscilation period

Ixtot = m*g*Txtot^2*strrad^2/(4*pi^2*0.55);%motor on the side
Iytot = m*g*Tytot^2*strrad^2/(4*pi^2*0.55);%motor on top

%from InnerGimbalAssembly_mobile
zcgpx = 0.0305;%m z distance from pivot to cg of 2-axis gimbal assembly
Ixxp_cg = 0.0007162665;%kg*m^2 moment of inertia about cg
mpx = 0.28867;%kg

% from InnerGimbalAssembly_mobile_y
zcgpy = 0.03495;%m z distance from pivot to cg of 1-axis gimbal assembly
Iyyp_cg = 0.000701359;%kg*m^2; moment of inertia about cg
mpy = 0.25211;%kg

%parallel axis theorem to get I about pivot points
Ixp = Ixxp_cg + zcgpx^2*mpx;
Iyp = Iyyp_cg + zcgpy^2*mpy;


F = m*g;%Thrust from propeller
s = tf('s');

G = (d*F +s^2*Ixp)/(s^2*Ixtot); % Attitude Dynamics
H = F/(m*s^2); % Translation Dynamics
J = 1/(m*s^2);% Altitude Dynamics TODO


%% Attitude controller design
%RR page 10-9
% The frequency at which the closed-loop gain falls below 0.7, and thus the output ceases to track the reference input faithfully, is the bandwidth, ωBW .

% From SLC Example in RR 10-31
tr = 0.5; % Rise time
ts = 2.3; % Settling time
M = 0.15; % Max overshoot
% eq8.17
wn = 1.8/tr; % Natural frequency
sigma = 4.6/ts;
% zeta = sigma/wn; % Damping ratio
zeta = 0.7;
% wd = wn*sqrt(1-zeta^2);% Damped natural frequency
%eq 10.8
wbw = 1.4*wn; % Bandwidth frequency
wg = wn; % Crossover frequency
% wp = 1; % Phase crossover frequency
alpha_lead = 9;
alpha_lag = 100;

% p = -12;
% z = -1.5;

% p = -10*wg^2;
% z = p/10;

% p = -sqrt(10*wg^2);
% z = p/10;

p = -wg*sqrt(alpha_lead);
z = -wg/sqrt(alpha_lead);
% plead = wg*sqrt(alpha_lead);
% zlead = wg/sqrt(alpha_lead);
% plag =  wg*sqrt(alpha_lag);
% zlag =  wg/sqrt(alpha_lag);

% K = 3.84;% From root locus
K = 1.34;
Dg = (s-z)/(s-p);% Lead-Lag attitude controller
% Dg = ((s+zlead)/(s+plead))*(s+zlag)/(s+plag);
DGc = feedback(K*Dg*G,1); % closed loop

%% Position (x) Controller Design
tr = 10; % Rise time
ts = 15; % Settling time
M = 0.05; % Max overshoot
% eq8.17
wnh = 1.8/tr; % Natural frequency
sigma = 4.6/ts;
% zeta = sigma/wn; % Damping ratio
zetah = 0.5;
wd = wn*sqrt(1-zeta^2);% Damped natural frequency
%eq 10.8
wbw = 1.4*wnh; % Bandwidth frequency
wgh = wnh; % Crossover frequency
% wp = 1; % Phase crossover 

ph = -1;
zh = -0.1;
% p = -sqrt(10*wg^2);
% z = p/10;
Kh = 0.32;% From root locus
% Kh = 0.796;

Dh = (s-zh)/(s-ph);% Lead-Lag position controller
DHc = feedback(Kh*Dh*H,1); % closed loop


%% Plotting

% % Plot Open Bode and Root Locus of G
% % fig1 = figure(1);
% % bode(G)
% % fig2 = figure(2);
% % rlocus(G)
% % step(G)
% 
% Plot Bode and Root Locus of GDh
fig3 = figure(3);
bodeg = bodeplot(K*Dg*G);
% test2= getoptions(test);
% test2.Ylim = {[-30, 100],[-190 300]};
% setoptions(test,test2);
% 
fig4 = figure(4);
rlocus(G*Dg)
sgrid(zeta,wn)
title(['Root Locus of G*Dg using lead-lag controller with p = ', num2str(p), ', z = ', num2str(z)])
% 
figure(5)
step(DGc)
% 
% figure(6)
% impulse(DGc)



% Plot Open Bode and Root Locus of H
% fig7 = figure(7);
% bode(H)
% fig8 = figure(8);
% rlocus(H)
% % step(H)

% % Plot Bode and Root Locus of HDh
% fig9 = figure(9);
% bodeh = bodeplot(Kh*Dh*H);
% % test2 = getoptions(test);
% % test2.Ylim = {[-30, 100],[-190 300]};
% % setoptions(test,test2);
% 
% fig10 = figure(10);
% rlocus(H*Dh)
% sgrid(zetah,wnh)
% title(['Root Locus of H*Dh using lead-lag controller with p = ', num2str(ph), ', z = ', num2str(zh)])
% 
% figure(11)
% step(DHc)
% 
% % figure(12)
% % impulse(DHc)

