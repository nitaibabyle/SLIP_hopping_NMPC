function [bianjie_tiaojian]=opt_boundary_constraint(MMM) 
global z_0 z_ed
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


a=[0,0,0,0,0,0;
   0,1,0,0,0,0;
   0,0,1,0,0,0;
   0,0,0,0,0,0;
   0,0,0,0,1,0;
   0,0,0,0,0,1];

b=[0,0,0,0,0,0;
   0,0,0,0,0,0;
   0,0,0,0,0,0;
   0,0,0,0,0,0;
   0,0,0,0,0,0;
   0,0,0,0,0,0];



bianjie_tiaojian=[b*(Qdd_Qd_q_t10-z_ed');a*(Qdd_Qd_q_t1-z_0')];
              
           