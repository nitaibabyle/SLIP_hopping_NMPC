function f=objective(MMM)
 global ed_dx ed_dy ed_y
ut=MMM(61:80);
time=MMM(71);
Qdd_Qd_q_t1=MMM(1:6);
Qdd_Qd_q_t2=MMM(7:12);
Qdd_Qd_q_t3=MMM(13:18);
Qdd_Qd_q_t4=MMM(19:24);
Qdd_Qd_q_t5=MMM(25:30);
Qdd_Qd_q_t6=MMM(31:36);
Qdd_Qd_q_t7=MMM(37:42);
Qdd_Qd_q_t8=MMM(43:48);
Qdd_Qd_q_t9=MMM(49:54);
Qdd_Qd_q_t10=MMM(55:60);
zzz(1,:)=[Qdd_Qd_q_t1(3),Qdd_Qd_q_t1(2),Qdd_Qd_q_t1(6),Qdd_Qd_q_t1(5)];
zzz(2,:)=[Qdd_Qd_q_t2(3),Qdd_Qd_q_t2(2),Qdd_Qd_q_t2(6),Qdd_Qd_q_t2(5)];
zzz(3,:)=[Qdd_Qd_q_t3(3),Qdd_Qd_q_t3(2),Qdd_Qd_q_t3(6),Qdd_Qd_q_t3(5)];
zzz(4,:)=[Qdd_Qd_q_t4(3),Qdd_Qd_q_t4(2),Qdd_Qd_q_t4(6),Qdd_Qd_q_t4(5)];
zzz(5,:)=[Qdd_Qd_q_t5(3),Qdd_Qd_q_t5(2),Qdd_Qd_q_t5(6),Qdd_Qd_q_t5(5)];
zzz(6,:)=[Qdd_Qd_q_t6(3),Qdd_Qd_q_t6(2),Qdd_Qd_q_t6(6),Qdd_Qd_q_t6(5)];
zzz(7,:)=[Qdd_Qd_q_t7(3),Qdd_Qd_q_t7(2),Qdd_Qd_q_t7(6),Qdd_Qd_q_t7(5)];
zzz(8,:)=[Qdd_Qd_q_t8(3),Qdd_Qd_q_t8(2),Qdd_Qd_q_t8(6),Qdd_Qd_q_t8(5)];
zzz(9,:)=[Qdd_Qd_q_t9(3),Qdd_Qd_q_t9(2),Qdd_Qd_q_t9(6),Qdd_Qd_q_t9(5)];
zzz(10,:)=[Qdd_Qd_q_t10(3),Qdd_Qd_q_t10(2),Qdd_Qd_q_t10(6),Qdd_Qd_q_t10(5)];

%%% nl_opt traj

xt=zzz(:,1);
dxt=zzz(:,2);
yt=zzz(:,3);
dyt=zzz(:,4);

a=0;
b=0;
c=0;
d=0;
e=0;

% for i=1:9
%     a=(9/2)*(ut(i)^2+ut(i+1)^2)+a;
%     b=(9/2)*((xt(i)-ed_x)^2+(xt(i+1)-ed_x)^2)+b;
%     c=(9/2)*(dxt(i)^2+dxt(i+1)^2)+c;
%     d=(9/2)*((thetat(i)-ed_theta)^2+(thetat(i+1)-ed_theta)^2)+d;
%     e=(9/2)*(dthetat(i)^2+dthetat(i+1)^2)+e;
%     
% end
for i=1:10
    a=ut(i)^2+ut(10+i)^2+a;
    b=(dxt(i)-ed_dx)^2+b;
    %c=(dyt(i)-ed_dy)^2+c;
    %d=(yt(i)-ed_y)^2+d;
end

f=a+100*b;



