function Jp = codegen_jacobianPoint_2(in1,in2)
%CODEGEN_JACOBIANPOINT_2
%    JP = CODEGEN_JACOBIANPOINT_2(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 00:22:01

%Input order: Q, r
q_1 = in1(1,:);
q_2 = in1(2,:);
r_1 = in2(1,:);
r_2 = in2(2,:);
r_3 = in2(3,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = conj(r_1);
t5 = conj(r_2);
t6 = conj(r_3);
t7 = sin(q_1);
t8 = sin(q_2);
t9 = t3.*2.7e-1;
t10 = t8.*2.7e-1;
t11 = t5.*t8;
t12 = t3.*t4.*-1.0;
t13 = t3.*t4.*1.0;
t14 = t9+t11+t12;
Jp = reshape([t7.*-7.5e-2-t2.*t6.*1.0-t7.*t8.*2.7e-1+t3.*t5.*t7+t4.*t7.*t8,t6.*t7.*-1.0+t2.*(t10+7.5e-2)-t2.*t3.*t5.*1.0-t2.*t4.*t8.*1.0,0.0,0.0,0.0,1.0,t2.*t14,t7.*t14,-t10+t3.*t5+t4.*t8,t7.*-1.0,t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);
