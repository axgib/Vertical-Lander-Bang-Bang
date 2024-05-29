% RR textbook: http://robotics.ucsd.edu/RR.pdf


close all

% Define phisical properties of system
g = 9.81;% m/s^2

% Measurement of moment of inertia: https://www.youtube.com/watch?v=nwgd1CV__rs&t=342s
m = 1.315; % kg
d = 0.143;%distance from gimbal pivot to cg_tot
strrad = 0.08255;% m
strlen = 0.55;% m
Txtot = 32.35/10; % oscilation period
Tytot = 29.3/10; % oscilation period

Ixtot = m*g*Txtot^2*strrad^2/(4*pi^2*0.55);%motor on the side
Iytot = m*g*Tytot^2*strrad^2/(4*pi^2*0.55);%motor on top

% from InnerGimbalAssembly_mobile
zcgpx = 0.0305;%m z distance from pivot to cg of 2-axis gimbal assembly
Ixxp_cg = 0.0007162665;%kg*m^2 moment of inertia about cg
mpx = 0.28867;%kg

% from InnerGimbalAssembly_mobile_y
zcgpy = 0.03495;%m z distance from pivot to cg of 1-axis gimbal assembly
Iyyp_cg = 0.000701359;%kg*m^2; moment of inertia about cg
mpy = 0.25211;%kg

% parallel axis theorem to get I about pivot points
Ixp = Ixxp_cg + zcgpx^2*mpx;
Iyp = Iyyp_cg + zcgpy^2*mpy;

tau = sqrt(d/g);% time constant

F = m*g;%Thrust from propeller
s = tf('s');

G_a = (d*F +s^2*Ixp)/(s^2*Ixtot); % Attitude Dynamics: fast inner loop
G_x = F/(m*s^2); % Translation Dynamics: slow outer loop
J = 1/(m*s^2);% Altitude Dynamics TODO

%% Attitude Controller
tr_a = 2.2*tau;% rise time requirement
zeta = 0.7;% damping ratio from max overshoot (RR Eq. 10.8)
wn_a = 1.8/tr_a;
boost = 10;% account for shift in wg with lead compensator
PM = 100*zeta + boost;% (RR Eq. 10.8)

PM1_a = PM/2;% create half of phase margin with 1st lead compensator
a1 = (1+sind(PM1_a))/(1-sind(PM1_a));% https://www.youtube.com/watch?v=rH44ttR3G4Q time: 10:50
b1 = 1/(wn_a*sqrt(a1));

D1_a = (a1*b1*s+1)/(b1*s+1);

figure(1)% Root Locus for uncontrolled G1
hold on
rlocus(G_a)
title('Root Locus of Uncontrolled G_a ')

figure(2)% Root Locus for 1st lead compensator
rlocus(D1_a*G_a)
sgrid(zeta,wn_a)
title('Root Locus of G_a with One Lead Controller')

%from fig(2)
K1_a = 0.525;

figure(3)% Open-Loop Bode for 1st lead compensator
margin(K1_a*D1_a*G_a)
title('Open-Loop Bode of G_a with One Lead Controller')

%from fig(3)
wn2_a = 6.97;% rad/s
PM1true_a = 40;% degrees

PM2_a = PM - PM1true_a;% create rest of phase margin with compensator 2
a2 = (1+sind(PM2_a))/(1-sind(PM2_a));
b2 = 1/(wn2_a*sqrt(a2));

D2_a = (a2*b2*s+1)/(b2*s+1);

figure(4) % Root Locus for 2nd lead compensator
rlocus(D2_a*K1_a*D1_a*G_a)
sgrid(zeta,wn_a)
title('Root Locus of G_a with Two Lead Controller')

%from fig(4)
K2_a = 0.7;
D_a = K2_a*D2_a*K1_a*D1_a;

figure(5) % Open-Loop Bode for 2nd lead compensator
margin(D_a*G_a)
title('Open-Loop Bode of G_a with Two Lead Controller')

%from fig(5)
PM2true_a = 73.7;% degrees

G_x_slc  = feedback(D_a*G_a,1); % Dynamics of inner loop of Successive Loop Closure (RR 10.3.4)
a_effort = D_a/(1 + D_a*G_a);% TODO: What units? degrees or rad?

figure(6) % Step Response
step(G_x_slc)
SG_a = stepinfo(G_x_slc);
title('Step Response of Attitude Dynamics')

figure(7)
step(a_effort)
title('Controller Effort for Step Response of Attitude Dynamics')
S_a_effort = stepinfo(a_effort);

%% Position Controller

tr_x = 20*SG_a.RiseTime;
zeta = 0.5;%1
wn_x = 1.8/tr_x;%0.3;
Kp = wn_x^2/(F/m);
Kd = 2*zeta*wn_x/(F/m);
wpd = 100;
D = Kp + Kd*s/(1+s/wpd);
K = 1;
D_x = K*D;
pd_closed = feedback(K*D*G_x,1);
S = stepinfo(pd_closed)

C = 0.0546 + 1.23/s;% from PID tuner on G_x_slc

x_effort = D_x/(1+D_x*G_x);

figure
hold on
rlocus(D*G_x);
sgrid(zeta,wn_x)
hold off

figure
margin(K*D*G_x)

figure
step(feedback(K*D*G_x,1))

figure
step(x_effort)

figure
hold on
rlocus(C*G_x_slc);
sgrid(zeta,wn_x)
hold off

figure
margin(C*G_x_slc)

figure
step(feedback(C*G_x_slc,1))



% figure
% step(feedback(K*D*G_x_slc,1))

