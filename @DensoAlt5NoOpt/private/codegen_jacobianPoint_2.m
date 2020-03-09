function Jp = codegen_jacobianPoint_2(in1,in2)
%CODEGEN_JACOBIANPOINT_2
%    JP = CODEGEN_JACOBIANPOINT_2(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 13:15:01

%Input order: Q, r
q_1 = in1(1,:);
q_2 = in1(2,:);
r_1 = in2(1,:);
r_2 = in2(2,:);
r_3 = in2(3,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = sin(q_1);
t5 = sin(q_2);
t6 = t3.*2.7e-1;
t7 = r_2.*t5;
t8 = r_1.*t3.*-1.0;
t9 = r_1.*t3.*1.0;
t10 = t6+t7+t8;
Jp = reshape([t4.*-7.5e-2-r_3.*t2.*1.0-t4.*t5.*2.7e-1+r_2.*t3.*t4+r_1.*t4.*t5,t2.*7.5e-2-r_3.*t4.*1.0+t2.*t5.*2.7e-1-r_2.*t2.*t3.*1.0-r_1.*t2.*t5.*1.0,0.0,0.0,0.0,1.0,t2.*t10,t4.*t10,t5.*-2.7e-1+r_2.*t3+r_1.*t5,t4.*-1.0,t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,5]);
