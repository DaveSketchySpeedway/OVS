% NOTATIONS
% r = roll velocty
% p = pitch velocity
% q = yaw velocity
% u = longitudinal velocity fwd pos
% v = lateral velocity left pos
% w = vertical velocity up pos
% x = global x
% y = global y
% z = global z up pos
% alpha_f = front tire slip angle
% alpha_r = rear tire slip angle
% delta_r = road wheel steer angle (reference steer angle)
% delta_h = hand wheel steer angle (steering angle)
% beta = slide slip angle = lateral velocity / longitudinal velocity
%
%
%


clear
close all
clc

m = 1600; % car mass (kg)
i_zz = 2830; % yaw inertia (kgm2)
a = 1.030; % distance from c.g. to front axle (m)
b = 1.717; % distanc from c.g. to rear axle (m)
d_f = 6/180*pi(); % front cornering compliance (rad/g)
d_r = 3/180*pi(); % rear cornering compliance (rad/g)
% m = 4000/2.2; % car mass (kg)
% i_zz = 25*0.304^2*m; % yaw inertia (kgm2)
% a = 5*0.304; % distance from c.g. to front axle (m)
% b = 5*0.304; % distanc from c.g. to rear axle (m)
% d_f = 12/180*pi(); % front cornering compliance (rad/g)
% d_r = 6/180*pi(); % rear cornering compliance (rad/g)
hpr = 16; % hand wheel / road wheel gear ratio
g = 9.81; % gravity (m/s2)
u = 100/3.6; % forward velocity (m/s)

wb = a + b; % wheelbase (m)
uog = d_f - d_r; % understeer gradient (rad/g)
u_ch = sqrt(g*wb/uog);   
m_f = m * b / wb; % front axle mass (kg)
m_r = m * a / wb; % rear axle mass (kg)
c_f = m_f * g / d_f * (pi()/180) /2; % front tire cornering stiffness (N/deg)
c_r = m_r * g / d_r * (pi()/180) /2; % rear tire cornering stiffness (N/deg)

%% final values
gain_refsteer_yawvel = u / (wb + u^2*uog/g); % rad/s / rad
gain_refsteer_latacc = u^2 / (wb + u^2*uog/g); %  m/s2 / rad
gain_refsteer_sideslip = (b - d_r*u^2/g) / (wb + u^2*uog/g); % r / r
gain_latacc_sideslip = b/u^2 - d_r/g; % rad / m/s2

check_beta_g = gain_latacc_sideslip*g/pi()*180 % deg / G
check_beta_d = gain_refsteer_sideslip*100/hpr % deg / 100 deg hand
check_ay_d = gain_refsteer_latacc/g*pi()/180*100/hpr % G / deg hand
check_r_d = gain_refsteer_yawvel*100/hpr % deg/s / 100 deg hand

clear check_*

%% transfer functions
n2 = 0;
n1 = g*a*b*u/d_f/wb;
n0 = g^2*a*b/d_f/d_r/wb;
d2 = i_zz*u/m;
d1 = g*(a^2*b*m*d_r + a*d_f*(b^2*m+i_zz) + b*d_r*i_zz)/d_f/d_r/m/wb;
d0 = g*a*b*(g*wb+u^2*uog)/d_f/d_r/u/wb;
plant_refsteer_yawvel.numc = [n2,n1,n0];
plant_refsteer_yawvel.denc = [d2,d1,d0];
plant_refsteer_yawvel.tfc = tf(...
    plant_refsteer_yawvel.numc, plant_refsteer_yawvel.denc);
clear n2 n1 n0 d2 d1 d0

n2 = g*b*u*i_zz/d_f/m/wb;
n1 = g^2*a*b^2/d_f/d_r/wb;
n0 = g^2*a*b*u/d_f/d_r/wb;
d2 = i_zz*u/m;
d1 = g*(a^2*b*m*d_r + a*d_f*(b^2*m+i_zz) + b*d_r*i_zz)/d_f/d_r/m/wb;
d0 = g*a*b*(g*wb+u^2*uog)/d_f/d_r/u/wb;
plant_refsteer_latacc.numc = [n2,n1,n0];
plant_refsteer_latacc.denc = [d2,d1,d0];
plant_refsteer_latacc.tfc = tf(...
    plant_refsteer_latacc.numc, plant_refsteer_latacc.denc);
clear n2 n1 n0 d2 d1 d0

n2 = 0;
n1 = g*b*i_zz/d_f/m/wb;
n0 = g*a*b*(g*b-d_r*u^2)/d_f/d_r/u/wb;
d2 = i_zz*u/m;
d1 = g*(a^2*b*m*d_r + a*d_f*(b^2*m+i_zz) + b*d_r*i_zz)/d_f/d_r/m/wb;
d0 = g*a*b*(g*wb+u^2*uog)/d_f/d_r/u/wb;
plant_refsteer_sideslip.numc = [n2,n1,n0];
plant_refsteer_sideslip.denc = [d2,d1,d0];
plant_refsteer_sideslip.tfc = tf(...
        plant_refsteer_sideslip.numc, plant_refsteer_sideslip.denc);
clear n2 n1 n0 d2 d1 d0

n2 = 0;
n1 = i_zz*u*d_r;
n0 = a*m*(g*b-d_r*u^2);
d2 = i_zz*u^2*d_r;
d1 = g*a*b*m*u;
d0 = g*a*m*u^2;
plant_latacc_sideslip.numc = [n2,n1,n0];
plant_latacc_sideslip.denc = [d2,d1,d0];
plant_latacc_sideslip.tfc = tf(...
    plant_latacc_sideslip.numc, plant_latacc_sideslip.denc);
clear n2 n1 n0 d2 d1 d0

%% step
figure
subplot(2,2,1)
step(plant_refsteer_yawvel.tfc);
title('refsteer to yawvel')
grid on

subplot(2,2,2)
step(plant_refsteer_latacc.tfc);
title('refsteer to latacc');
grid on

subplot(2,2,3)
step(plant_refsteer_sideslip.tfc);
title('refsteer to sideslip');
grid on

subplot(2,2,4)
step(plant_latacc_sideslip.tfc);
title('latacc to sideslip')
grid on

