function T = codegen_FK_1(in1)
%CODEGEN_FK_1
%    T = CODEGEN_FK_1(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 00:21:04

%Input order: Q.
q_1 = in1(1,:);
t2 = cos(q_1);
t3 = sin(q_1);
T = reshape([t2,t3,0.0,0.0,0.0,0.0,-1.0,0.0,t3.*-1.0,t2,0.0,0.0,t2.*7.5e-2,t3.*7.5e-2,1.32e-1,1.0],[4,4]);
