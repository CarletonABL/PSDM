function J = codegen_jacobian_1(in1)
%CODEGEN_JACOBIAN_1
%    J = CODEGEN_JACOBIAN_1(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    11-Feb-2020 15:36:08

%Input order: Q
q_1 = in1(1,:);
J = reshape([sin(q_1).*-7.5e-2,cos(q_1).*7.5e-2,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,4]);
