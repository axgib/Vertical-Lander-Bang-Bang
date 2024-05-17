%RR textbook: http://robotics.ucsd.edu/RR.pdf

% SSH Key: 192.168.7.2

close all
addpath(genpath('C:\Users\15103\MAE144_RR\RR'))

g = 9.81;
Txtot = 32.35/10;% oscilation period
Tytot = 29.3/10;%oscilation period
strrad = 0.08255;% m
strlen = 0.55;% m

m = 1.315; %kg
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

d = 0.2;

F = m*g;
s = tf('s');

G = (d*F +s^2*Ixp)/(s^2*Ixtot); % Attitude Dynamics
H = F/(m*s^2); % Translation Dynamics
% M = 1;
J = 1/(m*s^2);% Altitude Dynamics TODO: Check


%% Attitude controller design
%RR page 10-9
% The frequency at which the closed-loop gain falls below 0.7, and thus the output ceases to track the reference input faithfully, is the bandwidth, ωBW .

% From SLC Example in RR 10-31
tr = 0.5; %Rise time
ts = 2.3;
M = 0.10;
% eq8.17
wn = 1.8/tr; %natural frequency
sigma = 4.6/ts;
% zeta = sigma/wn; %damping ratio
zeta = 0.7;
wd = wn*sqrt(1-zeta^2);
%eq 10.8
wbw = 1.4*wn;
wg = wn; %Crossover frequency
wp = 1; %phase crossover frequency

p = -12;
z = -1.5;
% p = -sqrt(10*wg^2);
% z = p/10;
K = 0.844;

D = (s-z)/(s-p);% Lead-Lag controller
closed = feedback(K*D*G,1);%closed loop

%% Plotting

% Plot Open Bode and Root Locus of G
% fig1 = figure(1);
% bode(G)
% fig2 = figure(2);
% rlocus(G)
% step(G)

% Plot Bode and Root Locus of GD
fig3 = figure(3);
test = bodeplot(K*D*G);
% test2= getoptions(test);
% test2.Ylim = {[-30, 100],[-190 300]};
% setoptions(test,test2);

fig4 = figure(4);
rlocus(G*D)
sgrid(zeta,wn)
title(['Root Locus of G*D using lead-lag controller with p = ', num2str(p), ', z = ', num2str(z)])

figure(5)
step(closed)

figure(6)
impulse(closed)




