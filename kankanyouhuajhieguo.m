Qdd_Qd_q_t1=ccc(1:6);
Qdd_Qd_q_t2=ccc(7:12);
Qdd_Qd_q_t3=ccc(13:18);
Qdd_Qd_q_t4=ccc(19:24);
Qdd_Qd_q_t5=ccc(25:30);
Qdd_Qd_q_t6=ccc(31:36);
Qdd_Qd_q_t7=ccc(37:42);
Qdd_Qd_q_t8=ccc(43:48);
Qdd_Qd_q_t9=ccc(49:54);
Qdd_Qd_q_t10=ccc(55:60);
T=0.5;
g = 10;
l=1;
m = 1;
M = 10;

z(1,:)=[Qdd_Qd_q_t1(3),Qdd_Qd_q_t1(2),Qdd_Qd_q_t1(6),Qdd_Qd_q_t1(5)];
z(2,:)=[Qdd_Qd_q_t2(3),Qdd_Qd_q_t2(2),Qdd_Qd_q_t2(6),Qdd_Qd_q_t2(5)];
z(3,:)=[Qdd_Qd_q_t3(3),Qdd_Qd_q_t3(2),Qdd_Qd_q_t3(6),Qdd_Qd_q_t3(5)];
z(4,:)=[Qdd_Qd_q_t4(3),Qdd_Qd_q_t4(2),Qdd_Qd_q_t4(6),Qdd_Qd_q_t4(5)];
z(5,:)=[Qdd_Qd_q_t5(3),Qdd_Qd_q_t5(2),Qdd_Qd_q_t5(6),Qdd_Qd_q_t5(5)];
z(6,:)=[Qdd_Qd_q_t6(3),Qdd_Qd_q_t6(2),Qdd_Qd_q_t6(6),Qdd_Qd_q_t6(5)];
z(7,:)=[Qdd_Qd_q_t7(3),Qdd_Qd_q_t7(2),Qdd_Qd_q_t7(6),Qdd_Qd_q_t7(5)];
z(8,:)=[Qdd_Qd_q_t8(3),Qdd_Qd_q_t8(2),Qdd_Qd_q_t8(6),Qdd_Qd_q_t8(5)];
z(9,:)=[Qdd_Qd_q_t9(3),Qdd_Qd_q_t9(2),Qdd_Qd_q_t9(6),Qdd_Qd_q_t9(5)];
z(10,:)=[Qdd_Qd_q_t10(3),Qdd_Qd_q_t10(2),Qdd_Qd_q_t10(6),Qdd_Qd_q_t10(5)];

xx=z(:,1);
yy=z(:,3);



t=0:T/9:T;
x=interp1(t,xx,0:0.1:T,'cubic');
y=interp1(t,yy,0:0.1:T,'cubic');
time=interp1(t,t,0:0.1:T,'cubic');


figure(1)
for i=1:length(time)
    
    window_xmin = -2; window_xmax = 5;
    window_ymin = -0.6; window_ymax = 2;

    plot(x(i),y(i),'ko','MarkerEdgeColor','black', 'MarkerFaceColor',[0.5,0.5,0.5],'MarkerSize',10); %com
    line([x(i) 0],[y(i) 0],'Linewidth',4,'Color',[0.8 0 0]); %leg
    %hold on 
   
%hold off
    %plot(z(i,1)+ip.l*sin(z(i,3)),-ip.l*cos(z(i,3)),'ro','MarkerEdgeColor','black', 'MarkerFaceColor','black','MarkerSize',10); %com
    %hold off
   
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    %pause(0.05);
    F(i)=getframe;
end


