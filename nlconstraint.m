function [c,ceq]=nlconstraint(MMM)
c=opt_budengshi_constraint(MMM);
ceq=[opt_dynamic_constraint(MMM);...
    opt_discrete_constraint(MMM);...
    opt_boundary_constraint(MMM)];
    %MMM(366)+50;MMM(367)+50;MMM(368)+50;MMM(369)+50;MMM(370)+50];

