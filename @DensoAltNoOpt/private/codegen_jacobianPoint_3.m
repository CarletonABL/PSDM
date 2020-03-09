function Jp = codegen_jacobianPoint_3(in1,in2)
%CODEGEN_JACOBIANPOINT_3
%    JP = CODEGEN_JACOBIANPOINT_3(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 07:30:30

%Input order: Q, r
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
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
t9 = q_2+q_3;
t10 = t3.*2.7e-1;
t11 = t7.*-1.0;
t12 = t7.*1.0;
t13 = t8.*2.7e-1;
t14 = cos(t9);
t15 = sin(t9);
t16 = t14.*-9.0e-2;
t17 = t14.*9.0e-2;
t18 = t15.*9.0e-2;
t19 = t6.*t14;
t20 = t4.*t14.*-1.0;
t21 = t4.*t14.*1.0;
t22 = t4.*t15.*-1.0;
t23 = t4.*t15.*1.0;
t24 = t6.*t15.*-1.0;
t25 = t6.*t15.*1.0;
t26 = t13+t16+7.5e-2;
t27 = t18+t19+t22;
t28 = t10+t27;
Jp = reshape([t2.*t5.*-1.0+t11.*t26+t4.*t11.*t14+t6.*t11.*t15,t5.*t11+t2.*t26+t2.*t4.*t14+t2.*t6.*t15,0.0,0.0,0.0,1.0,t2.*t28,t7.*t28,-t13+t17+t20+t24,t11,t2,0.0,t2.*t27,t7.*t27,t17+t20+t24,t11,t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);
