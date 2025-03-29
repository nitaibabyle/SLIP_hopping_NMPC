function [budengshi_constraint]=opt_budengshi_constraint(MMM) 

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
ut=MMM(61:80);
time=MMM(81);

X1=[Qdd_Qd_q_t1;ut(1);ut(11)];
X2=[Qdd_Qd_q_t2;ut(2);ut(12)];
X3=[Qdd_Qd_q_t3;ut(3);ut(13)];
X4=[Qdd_Qd_q_t4;ut(4);ut(14)];
X5=[Qdd_Qd_q_t5;ut(5);ut(15)];
X6=[Qdd_Qd_q_t6;ut(6);ut(16)];
X7=[Qdd_Qd_q_t7;ut(7);ut(17)];
X8=[Qdd_Qd_q_t8;ut(8);ut(18)];
X9=[Qdd_Qd_q_t9;ut(9);ut(19)];
X10=[Qdd_Qd_q_t10;ut(10);ut(20)];

[budengshi_constraint1]=aaabudengshi_constraint(X1);
[budengshi_constraint2]=aaabudengshi_constraint(X2);
[budengshi_constraint3]=aaabudengshi_constraint(X3);
[budengshi_constraint4]=aaabudengshi_constraint(X4);
[budengshi_constraint5]=aaabudengshi_constraint(X5);
[budengshi_constraint6]=aaabudengshi_constraint(X6);
[budengshi_constraint7]=aaabudengshi_constraint(X7);
[budengshi_constraint8]=aaabudengshi_constraint(X8);
[budengshi_constraint9]=aaabudengshi_constraint(X9);
[budengshi_constraint10]=aaabudengshi_constraint(X10);

budengshi_constraint=[budengshi_constraint1;budengshi_constraint2;budengshi_constraint3;budengshi_constraint4;budengshi_constraint5;...
    budengshi_constraint6;budengshi_constraint7;budengshi_constraint8;budengshi_constraint9;budengshi_constraint10;-time];

