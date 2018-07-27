function ikin_sol = ikin(in1,l1,l2,l0)
%IKIN
%    IKIN_SOL = IKIN(IN1,L1,L2,L0)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    03-Apr-2018 17:49:56

start1 = in1(:,1);
start2 = in1(:,2);
start3 = in1(:,3);
t2 = start1.*1i;
t3 = start2+t2;
t4 = abs(t3);
t5 = l1-l2+t4;
t6 = 1.0./t5;
t7 = t4.^2;
t8 = l1.^2;
t9 = l2.^2;
t10 = l1.*t4.*2.0;
t11 = start3.^2;
t12 = t7+t8-t9+t10+t11;
t13 = sqrt(t12);
t14 = l0+t13;
t15 = l0+start3;
t16 = t6.*t15;
t17 = l0-t13;
ikin_sol = reshape([atan(t16-t6.*t14).*-2.0,atan(-t16+t6.*t17).*2.0,t14,t17],[2,2]);