function [chafen_yueshu]=opt_discrete_constraint(MMM)
global T
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

Qdd_t1=[Qdd_Qd_q_t1(1);Qdd_Qd_q_t1(4)];
Qd_t1=[Qdd_Qd_q_t1(2);Qdd_Qd_q_t1(5)];
Q_t1=[Qdd_Qd_q_t1(3);Qdd_Qd_q_t1(6)];

Qdd_t2=[Qdd_Qd_q_t2(1);Qdd_Qd_q_t2(4)];
Qd_t2=[Qdd_Qd_q_t2(2);Qdd_Qd_q_t2(5)];
Q_t2=[Qdd_Qd_q_t2(3);Qdd_Qd_q_t2(6)];

Qdd_t3=[Qdd_Qd_q_t3(1);Qdd_Qd_q_t3(4)];
Qd_t3=[Qdd_Qd_q_t3(2);Qdd_Qd_q_t3(5)];
Q_t3=[Qdd_Qd_q_t3(3);Qdd_Qd_q_t3(6)];

Qdd_t4=[Qdd_Qd_q_t4(1);Qdd_Qd_q_t4(4)];
Qd_t4=[Qdd_Qd_q_t4(2);Qdd_Qd_q_t4(5)];
Q_t4=[Qdd_Qd_q_t4(3);Qdd_Qd_q_t4(6)];

Qdd_t5=[Qdd_Qd_q_t5(1);Qdd_Qd_q_t5(4)];
Qd_t5=[Qdd_Qd_q_t5(2);Qdd_Qd_q_t5(5)];
Q_t5=[Qdd_Qd_q_t5(3);Qdd_Qd_q_t5(6)];

Qdd_t6=[Qdd_Qd_q_t6(1);Qdd_Qd_q_t6(4)];
Qd_t6=[Qdd_Qd_q_t6(2);Qdd_Qd_q_t6(5)];
Q_t6=[Qdd_Qd_q_t6(3);Qdd_Qd_q_t6(6)];

Qdd_t7=[Qdd_Qd_q_t7(1);Qdd_Qd_q_t7(4)];
Qd_t7=[Qdd_Qd_q_t7(2);Qdd_Qd_q_t7(5)];
Q_t7=[Qdd_Qd_q_t7(3);Qdd_Qd_q_t7(6)];

Qdd_t8=[Qdd_Qd_q_t8(1);Qdd_Qd_q_t8(4)];
Qd_t8=[Qdd_Qd_q_t8(2);Qdd_Qd_q_t8(5)];
Q_t8=[Qdd_Qd_q_t8(3);Qdd_Qd_q_t8(6)];

Qdd_t9=[Qdd_Qd_q_t9(1);Qdd_Qd_q_t9(4)];
Qd_t9=[Qdd_Qd_q_t9(2);Qdd_Qd_q_t9(5)];
Q_t9=[Qdd_Qd_q_t9(3);Qdd_Qd_q_t9(6)];

Qdd_t10=[Qdd_Qd_q_t10(1);Qdd_Qd_q_t10(4)];
Qd_t10=[Qdd_Qd_q_t10(2);Qdd_Qd_q_t10(5)];
Q_t10=[Qdd_Qd_q_t10(3);Qdd_Qd_q_t10(6)];

%Time=MMM(371);

time=MMM(71);
Time=T;
dt=Time/9;

chafen_yueshu=[Q_t2-Q_t1-(Qd_t1+Qd_t2)*dt/2;
               Q_t3-Q_t2-(Qd_t2+Qd_t3)*dt/2;
               Q_t4-Q_t3-(Qd_t3+Qd_t4)*dt/2;
               Q_t5-Q_t4-(Qd_t4+Qd_t5)*dt/2;
               Q_t6-Q_t5-(Qd_t5+Qd_t6)*dt/2;
               Q_t7-Q_t6-(Qd_t6+Qd_t7)*dt/2;
               Q_t8-Q_t7-(Qd_t7+Qd_t8)*dt/2;
               Q_t9-Q_t8-(Qd_t8+Qd_t9)*dt/2;
               Q_t10-Q_t9-(Qd_t9+Qd_t10)*dt/2;
               Qd_t2-Qd_t1-(Qdd_t1+Qdd_t2)*dt/2;
               Qd_t3-Qd_t2-(Qdd_t2+Qdd_t3)*dt/2;
               Qd_t4-Qd_t3-(Qdd_t3+Qdd_t4)*dt/2;
               Qd_t5-Qd_t4-(Qdd_t4+Qdd_t5)*dt/2;
               Qd_t6-Qd_t5-(Qdd_t5+Qdd_t6)*dt/2;
               Qd_t7-Qd_t6-(Qdd_t6+Qdd_t7)*dt/2;
               Qd_t8-Qd_t7-(Qdd_t7+Qdd_t8)*dt/2;
               Qd_t9-Qd_t8-(Qdd_t8+Qdd_t9)*dt/2;
               Qd_t10-Qd_t9-(Qdd_t9+Qdd_t10)*dt/2;];
           