clear all
clc
global l M g ed_y ed_x ed_dy ed_dx T kp
g = 9.8;
l=1;
M = 1;
kp=100;
% Initial conditions and other settings.
framespersec=1000;  %if view is not speeded or slowed in dbpend_animate
T=0.5;             %duration of animation  in seconds
time=T;
% 
x_ed=0.6;        dx_ed=0;      ddx_ed=0;
y_ed=0.8;        dy_ed=0;      ddy_ed=0;
ed_y=y_ed;
ed_x=x_ed;
ed_dy=dy_ed;
ed_dx=dx_ed;
%theta_ed=0.0573;    dtheta_ed=0;

x_0=-0;        dx_0=0.5;     ddx_0=0;
y_0=1;        dy_0=-0;     ddy_0=0;
%theta_0=0.0573;    dtheta_0=0;
%u0=[0,0,0,0,0,0,0,0,0,0];
u0=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];%10x,10y
global z_0 z_ed
z_0=[ddx_0,dx_0,x_0,ddy_0,dy_0,y_0];
z_ed=[ddx_ed,dx_ed,x_ed,ddy_ed,dy_ed,y_ed];

MMM0=[z_0';
    (8*z_0'+1*z_ed')/9;
    (7*z_0'+2*z_ed')/9;
    (6*z_0'+3*z_ed')/9;
    (5*z_0'+4*z_ed')/9;
    (4*z_0'+5*z_ed')/9;
    (3*z_0'+6*z_ed')/9;
    (2*z_0'+7*z_ed')/9; 
    (1*z_0'+8*z_ed')/9;
    z_ed';   u0';T];
      %Kp_spring1;Kp_spring2;Kp_spring3;Kp_spring4;Kp_spring5;Kd_spring1;Kd_spring2;Kd_spring3;Kd_spring4;Kd_spring5;Time0];
% MMM0=ccc;
% options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
options = optimoptions('fmincon',...
    'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
%     options = optimoptions('fmincon','Display','iter');
[ccc,y]=fmincon('objective',MMM0,[],[],[],[],[],[],'nlconstraint',options);



