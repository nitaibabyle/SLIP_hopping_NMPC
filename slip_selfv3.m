function slip

clc
clear all
close all

format long
slip.g = 10;
slip.ground = 0; 
slip.l = 1;
slip.m = 1;
slip.kp = 100;
slip.kd = 0;
slip.theta = 10*pi/180;
slip.v_ctrl=0;


global F_nmpc_store
F_nmpc_store=[0,0];

x0=1;xd0=0.5;y0=1.5;yd0=0;
z0=[x0 xd0 y0 yd0];

v_des=[0 0 0 0.8 0.8 0.8 0 0 -0.4 -0.4 -0.4 0 0 0 0];
slip.T = pi*sqrt(slip.m/slip.kp);
slip.footKp = 0.02;

t0=0;
dt=10;
t_ode=t0;
z_ode=[z0 x0 y0-slip.l 0 0];

steps=length(v_des);
fps = 2;
for i=1:steps
    v_release=z0(2);
    correction=slip.footKp*(v_release-v_des(i));
    slip.v_ctrl=v_des(i);
    slip.theta=asin((v_release*slip.T)/(2*slip.l))+correction;
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
    tspan = linspace(t0,t0+dt,dt*2000);
    [t_temp1,z_temp1] = ode113(@fly,tspan,z0,options1,slip);
    [t_temp1,z_temp1] = loco_interpolate(t_temp1,z_temp1,10*fps);
    z_temp1 = [z_temp1 ...
               z_temp1(1:end,1)+slip.l*sin(slip.theta) ...
               z_temp1(1:end,3)-slip.l*cos(slip.theta) ...
               0*ones(length(z_temp1),1) ...
               0*ones(length(z_temp1),1) ...
               ]
    t0=t_temp1(end);
    z0(1:4)=z_temp1(end,1:4);
    x_com=z0(1);
    z0(1)=-slip.l*sin(slip.theta);
    x_foot=x_com+slip.l*sin(slip.theta);
    y_foot=slip.ground;

options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
    tspan = linspace(t0,t0+dt,dt*2000);
    [t_temp2,z_temp2] = ode113(@spring,tspan,z0,options2,slip);
    [t_temp2,z_temp2] = loco_interpolate(t_temp2,z_temp2,10*fps);  

   % 将记载的补偿力插值在新产生的变量空间中
    t_nmpc_stored = linspace(t_temp2(1),t_temp2(end),length(F_nmpc_store));
    F_interp= interp1(t_nmpc_stored,F_nmpc_store,t_temp2);
    z_temp2(:,1)=z_temp2(:,1)+x_com+slip.l*sin(slip.theta);
    z_temp2 = [z_temp2 ...
               x_foot*ones(length(z_temp2),1) ...
               y_foot*ones(length(z_temp2),1)...
               F_interp...
               ];
           F_nmpc_store=[0,0];
           
            
% foot_y=[foot_y 0*ones(1,length(z_temp2))];
    t0=t_temp2(end);
    z0(1:4)=z_temp2(end,1:4);
    t_ode = [t_ode; t_temp1(2:end);t_temp2(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:);z_temp2(2:end,:)];
t=t_ode;
z=z_ode;

end

window_xmin = -1; window_xmax = 10;
window_ymin = -0.1; window_ymax = 3;

figure(1);
for i=1:length(t)
   
    %plot(z(i,1)+z(i,7),z(i,3)+z(i,8),'ro','MarkerEdgeColor','green','MarkerSize',5); %com
    plot(z(i,1),z(i,3),'ro','MarkerEdgeColor','black', 'MarkerFaceColor','black','MarkerSize',10); %com
    line([-10 10],[0 0],'Linewidth',2,'Color','black'); %ground
    line([z(i,1) z(i,5)],[z(i,3) z(i,6)],'Linewidth',4,'Color',[0.8 0 0]); %leg
    line([z(i,1) z(i,1)+z(i,7)],[z(i,3) z(i,3)+z(i,8)],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    
    %line([z(i,1)+z(i,7)+cos(0.5+atan(z(i,8)/z(i,7))) z(i,1)+z(i,7)+sin(0.5+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8)+cos(-0.5+atan(z(i,8)/z(i,7))) z(i,3)+z(i,8)+sin(-0.5+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0 0.8]); %leg
    
    
    if z(i,8)>=0&&z(i,7)>=0
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)-0.2*cos(0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)-0.2*sin(0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)-0.2*cos(-0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)-0.2*sin(-0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    end
    if z(i,8)>=0&&z(i,7)<=0
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)+0.2*cos(0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)+0.2*sin(0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)+0.2*cos(-0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)+0.2*sin(-0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    end
    if z(i,8)<=0&&z(i,7)>=0
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)-0.2*cos(0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)-0.2*sin(0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)-0.2*cos(-0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)-0.2*sin(-0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    end
    if z(i,8)<=0&&z(i,7)<=0
    
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)+0.2*cos(0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)+0.2*sin(0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    line([z(i,1)+z(i,7) z(i,1)+z(i,7)+0.2*cos(-0.3+atan(z(i,8)/z(i,7)))],[z(i,3)+z(i,8) z(i,3)+z(i,8)+0.2*sin(-0.3+atan(z(i,8)/z(i,7)))],'Linewidth',1,'Color',[0.0 0.8 0.0]); %leg
    end
    
    %annotation('arrow',[(z(i,1)-window_xmin)/11 (z(i,1)+z(i,7)-window_xmin)/11],[(z(i,3)-window_ymin)/3.1 ((z(i,3)+z(i,8))-window_ymin)/3.1]); %leg
    
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    %pause(0.05);
    F(i)=getframe;
end
v = VideoWriter('232.avi');
open(v);
writeVideo(v,F);
close(v);
% for i=1:20:length(tspan)
%     pause(.001)
%     plot(z(i,1),z(i,3),'ko','MarkerSize',3); %pivot point
%     hold on
%     plot([z(i,1) 0*ones(1,length(tspan))],[z(i,3) foot_y'],'b','LineWidth',2);% second pendulum
%     line([-2 2],[0 0],'Linewidth',2,'Color','black'); %ground
%     axis([-2*slip.l 2*slip.l -0.5 4*slip.l]);
%     axis square
%     hold off
 %    F(i)=getframe(gcf);
% end

function zdot = fly(t,z,slip)
zdot = [z(2) 0 z(4) -slip.g]';

% function zdot = spring(t,z,slip)
% x=z(1);xd=z(2);y=z(3);yd=z(4);
% l=sqrt(x^2+y^2);
% dl=yd*y/l+xd*x/l;
% F=slip.kp*(slip.l-l)-slip.kd*dl;
% Fx=F*x/l;
% Fy=F*y/l;
% xdd=Fx/slip.m;
% ydd=(Fy-slip.m*slip.g)/slip.m;
% zdot = [z(2) xdd z(4) ydd]';

function zdot=spring(t,z,slip)  
global F_nmpc_store
x=z(1);y=z(3);
l = sqrt(x^2+y^2);
F_spring = slip.kp*(slip.l-l);
Fx_spring =  F_spring*(x/l);
Fy_spring = F_spring*(y/l);
Fy_gravity = slip.m*slip.g;

Fx_nmpc=-3*(z(2)-slip.v_ctrl);
Fy_nmpc=-0.06*(z(4)-0);

%计算补偿力
% start_state=z;
% end_state=[0,slip.v_ctrl,0,0];
% slip.T
% opt_result=NMPC_traj(start_state, end_state, current_input);
% F_nmpc=opt_result;
% Fx_nmpc=F_nmpc(1);
% Fy_nmpc=F_nmpc(2);

%记载补偿力存到动力学的变量中
F_nmpc_store=[F_nmpc_store;Fx_nmpc,Fy_nmpc];

xdd = (1/slip.m)*(Fx_spring+Fx_nmpc);
ydd = (1/slip.m)*(-Fy_gravity+Fy_spring+Fy_nmpc);
zdot = [z(2) xdd z(4) ydd]';


function [gstop, isterminal,direction]=contact(t,z,slip)
y=z(3);
gstop = y-slip.l*cos(slip.theta);
direction = -1;
isterminal = 1;

function [gstop, isterminal,direction]=release(t,z,slip)
x=z(1);y=z(3);
gstop = sqrt(x^2+y^2)-slip.l;
direction = 1;
isterminal = 1;

function [t_interp,z_interp] = loco_interpolate(t_all,z_all,fps)
%===================================================================
[m,n] = size(z_all);
t_interp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));

for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),t_interp);
end
t_interp = t_interp';