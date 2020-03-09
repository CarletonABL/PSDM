function J = codegen_jacobian_3(in1)
%CODEGEN_JACOBIAN_3
%    J = CODEGEN_JACOBIAN_3(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 00:21:33

%Input order: Q
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = sin(q_1);
t5 = sin(q_2);
t6 = q_2+q_3;
t7 = t3.*2.7e-1;
t8 = t4.*-1.0;
t9 = t4.*1.0;
t10 = t5.*2.7e-1;
t11 = cos(t6);
t12 = sin(t6);
t13 = t11.*-9.0e-2;
t14 = t11.*9.0e-2;
t15 = t12.*9.0e-2;
t16 = t7+t15;
t17 = t10+t13+7.5e-2;
J = reshape([t8.*t17,t2.*t17,0.0,0.0,0.0,1.0,t2.*t16,t4.*t16,-t10+t14,t8,t2,0.0,t2.*t15,t4.*t15,t14,t8,t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);
