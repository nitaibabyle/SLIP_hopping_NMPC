function [dynamic]=aaadynamic_constraint(X)
global l M m g kp
ddx=X(1);
dx=X(2);
x=X(3);
ddy=X(4);
dy=X(5);
y=X(6);
ux=X(7);
uy=X(8);

l_act = sqrt(x^2+y^2);
F_spring = kp*(l-l_act);
Fx_spring =  F_spring*(x/l_act);
Fy_spring = F_spring*(y/l_act);
Fy_gravity = M*g;
xdd = ddx-(1/M)*(Fx_spring+ux);
ydd = ddy-(1/M)*(-Fy_gravity+Fy_spring+uy);

dynamic1=xdd;
dynamic2=ydd;
dynamic = [dynamic1;dynamic2];