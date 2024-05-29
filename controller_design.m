%RR textbook: http://robotics.ucsd.edu/RR.pdf

% SSH Key: 192.168.7.2

close all
addpath(genpath('C:\Users\15103\MAE144_RR\RR'))

%% Define phisical properties of system
g = 9.81;% m/s^2

%Measurement of moment of inertia: https://www.youtube.com/watch?v=nwgd1CV__rs&t=342s
m = 1.315; % kg
d = 0.143;%distance from gimbal pivot to cg_tot
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

tau = sqrt(d/g);% time constant

F = m*g;%Thrust from propeller
s = tf('s');

G = (d*F +s^2*Ixp)/(s^2*Ixtot); % Attitude Dynamics
H = F/(m*s^2); % Translation Dynamics
J = 1/(m*s^2);% Altitude Dynamics TODO


%% Attitude controller design
%RR page 10-9
% The frequency at which the closed-loop gain falls below 0.7, and thus the output ceases to track the reference input faithfully, is the bandwidth, ωBW .


% BW  = 1/(2*pi*tau);
tr_phi = 2.2*tau;
% tr = 0.5; % Rise time % From SLC Example in RR 10-31
ts_phi = 1; % Settling time
% eq8.17
wn_phi = 1.8/tr_phi; % Natural frequency
sigma = 4.6/ts_phi;
% zeta = sigma/wn; % Damping ratio
zeta_phi = 0.5;%from max overshoot M = 0.15 RR 10-9
% wd = wn*sqrt(1-zeta^2);% Damped natural frequency
%eq 10.8
wbw = 1.4*wn_phi; % Bandwidth frequency
wg_phi = wn_phi; % Crossover frequency
% wp = 1; % Phase crossover frequency
alpha_lead_phi = 9.4; % alpha = p/z 9 good
% alpha_lag = 100;% alpha = z/p

% p = -12;
% z = -1.5;

% p = -10*wg^2;
% z = p/10;

% p = -sqrt(10*wg^2);
% z = p/10;

p_lead_phi = -wg_phi*sqrt(alpha_lead_phi);
z_lead_phi = -wg_phi/sqrt(alpha_lead_phi);
% plead = wg*sqrt(alpha_lead);
% zlead = wg/sqrt(alpha_lead);
% plag =  wg*sqrt(alpha_lag);
% zlag =  wg/sqrt(alpha_lag);

% K = 3.84;% From root locus
K_phi = 3.16;% for alpha = 9, K = 3.1
D_phi = (s-z_lead_phi)/(s-p_lead_phi);% Lead-Lag attitude controller
D1doug = (0.1859*s+1)/(0.0664*s+1);
K1doug = 1.37;
D2doug = (0.11065*s + 1)/(0.0461*s + 1);
K2doug = 0.85;
% Dg = ((s+zlead)/(s+plead))*(s+zlag)/(s+plag);
DGc = feedback(K_phi*D_phi*G,1); % closed loop

%% Position (x) Controller Design
tr_x = 10*tr_phi; % Rise time
% ts = 15; % Settling time
% M = 0.05; % Max overshoot
% % eq8.17
% wnh = 1.8/tr; % Natural frequency
% sigma = 4.6/ts;
% % zeta = sigma/wn; % Damping ratio
zeta_x = 0.7; % Max overshoot of 0.05
% wd = wn*sqrt(1-zeta^2);% Damped natural frequency
% %eq 10.8
% wbw = 1.4*wnh; % Bandwidth frequency
% wgh = wnh; % Crossover frequency
% % wp = 1; % Phase crossover 
% 
% ph = -1;
% zh = -0.1;
% % p = -sqrt(10*wg^2);
% % z = p/10;
% Kh = 0.32;% From root locus
% % Kh = 0.796;g
% 
% Dh = (s-zh)/(s-ph);% Lead-Lag position controller
% DHc = feedback(Kh*Dh*H,1); % closed loop
% 

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
% bodeg = bodeplot(K*Dg*G);
margin(K_phi*D_phi*G)
% 
fig4 = figure(4);
rlocus(G*D_phi)
sgrid(zeta_phi,wn_phi)
title(['Root Locus of G*Dg using lead-lag controller with p = ', num2str(p_lead_phi), ', z = ', num2str(z_lead_phi)])

% Closed-Loop step response of D*G

figure(5)
hold on
step(DGc)
% xline(tr,'--')
yline(0.9, '--')
yline(1.02, '--')
hold off
SG = stepinfo(DGc);

figure(6)
hold on
margin(DGc)
% bodeg_closed = bodeplot(DGc);
hold off
figure(7)
hold on
rlocus(D1doug*G)
sgrid(0.7,7)
hold off
figure(8)
margin(feedback(K1doug*D1doug*G,1))

figure(9)
rlocus(D2doug*K1doug*D1doug*G)
sgrid(0.7,7)

figure(10)
margin(feedback(K2doug*D2doug*K1doug*D1doug*G,1))

figure(11)
step(feedback(K2doug*D2doug*K1doug*D1doug*G,1))
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













%% Old Position Controller from v2
tr_x = 10*tr_a;% slow dynamics
%use same zeta
wn_x = 1.8/tr_x;
%use same PM

% PM1_x = 55; %PM/2;% create half of phase margin with 1st lead compensator
% a1 = (1+sind(PM1_x))/(1-sind(PM1_x));% https://www.youtube.com/watch?v=rH44ttR3G4Q time: 10:50
% b1 = 1/(wn_x*sqrt(a1));
% 
% D1_x = (a1*b1*s+1)/(b1*s+1);
% 
% figure(7)% Root Locus for uncontrolled G2
% rlocus(G_x)
% 
% figure(8)% Bode for uncontrolled G2
% margin(G_x)
% 
% figure(9)% Root Locus for 1st lead compensator
% rlocus(D1_x*G_x)
% sgrid(zeta,wn_x)
% 
% %from fig(9)
% K1_x = 0.014;
% 
% figure(10)% Open-Loop Bode for 1st lead compensator
% margin(K1_x*D1_x*G_x)
% 
% %from fig(11)
% wn2_x = 0.947;% rad/s
% PM1true_x = 53.5;% degrees
% 
% PM2_x = PM - PM1true_x;% create rest of phase margin with compensator 2
% a2 = (1+sind(PM2_x))/(1-sind(PM2_x));
% b2 = 1/(wn2_x*sqrt(a2));
% 
% D2_x = (a2*b2*s+1)/(b2*s+1);
% 
% figure(12) % Root Locus for 2nd lead compensator
% rlocus(D2_x*K1_x*D1_x*G_x)
% sgrid(zeta,wn_x)
% 
% %from fig(12)
% K2_x = 1.15;
% 
% figure(13) % Open-Loop Bode for 2nd lead compensator
% margin(K2_x*D2_x*K1_x*D1_x*G_x)
% 
% %from fig(14)
% PM2true_x = 73.9;% degrees
% 
% outer_closed = feedback(K2_x*D2_x*K1_x*D1_x*G_x,1);
% 
% figure(15) % Step Response
% step(outer_closed)
% SG_x = stepinfo(outer_closed);
% 
% outer_slc = feedback(K2_x*D2_x*K1_x*D1_x*G_x_slc,1);
% 
% figure(16) % Step Response
% step(outer_slc)
% SG_slc = stepinfo(outer_slc);


