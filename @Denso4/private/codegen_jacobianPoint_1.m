function Jp = codegen_jacobianPoint_1(in1,in2)
%CODEGEN_JACOBIANPOINT_1
%    JP = CODEGEN_JACOBIANPOINT_1(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    11-Feb-2020 15:36:14

%Input order: Q, r
q_1 = in1(1,:);
r_1 = in2(1,:);
r_3 = in2(3,:);
t2 = cos(q_1);
t3 = sin(q_1);
Jp = reshape([t3.*-7.5e-2-r_1.*t3.*1.0-r_3.*t2.*1.0,t2.*7.5e-2+r_1.*t2-r_3.*t3.*1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,4]);
