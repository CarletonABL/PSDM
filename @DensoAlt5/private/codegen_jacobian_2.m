function J = codegen_jacobian_2(in1)
%CODEGEN_JACOBIAN_2
%    J = CODEGEN_JACOBIAN_2(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 11:33:59

%Input order: Q
q_1 = in1(1,:);
q_2 = in1(2,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = sin(q_1);
t5 = sin(q_2);
t6 = t5.*2.7e-1;
t7 = t6+7.5e-2;
J = reshape([t4.*t7.*-1.0,t2.*t7,0.0,0.0,0.0,1.0,t2.*t3.*2.7e-1,t3.*t4.*2.7e-1,-t6,t4.*-1.0,t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,5]);
