function Jt = codegen_jacobian_tool(in1,in2)
%CODEGEN_JACOBIAN_TOOL
%    JT = CODEGEN_JACOBIAN_TOOL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 11:34:02

%Input order: Q, dtool
d_tool_x = in2(1,:);
d_tool_y = in2(2,:);
d_tool_z = in2(3,:);
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
q_4 = in1(4,:);
q_5 = in1(5,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = cos(q_4);
t6 = cos(q_5);
t7 = sin(q_1);
t8 = sin(q_2);
t9 = sin(q_3);
t10 = sin(q_4);
t11 = sin(q_5);
t12 = q_2+q_3;
t13 = t3.*2.7e-1;
t14 = t7.*-1.0;
t15 = t7.*1.0;
t16 = sin(t12);
t17 = t3.*t4.*2.95e-1;
t18 = t3.*t9.*-2.95e-1;
t19 = t3.*t9.*2.95e-1;
t20 = t4.*t8.*-2.95e-1;
t21 = t4.*t8.*2.95e-1;
t22 = t3.*t4.*9.0e-2;
t23 = t8.*t9.*-2.95e-1;
t24 = t8.*t9.*2.95e-1;
t25 = t3.*t9.*9.0e-2;
t26 = t4.*t8.*9.0e-2;
t27 = t8.*t9.*-9.0e-2;
t28 = t8.*t9.*9.0e-2;
t29 = d_tool_z.*t3.*t4.*t6;
t30 = d_tool_y.*t3.*t4.*t10;
t31 = d_tool_x.*t3.*t9.*t11;
t32 = d_tool_x.*t4.*t8.*t11;
t33 = d_tool_y.*t3.*t9.*t10;
t34 = d_tool_y.*t4.*t8.*t10;
t35 = d_tool_x.*t8.*t9.*t11;
t36 = d_tool_x.*t3.*t4.*t11.*-1.0;
t37 = d_tool_x.*t3.*t4.*t11.*1.0;
t38 = d_tool_z.*t3.*t6.*t9.*-1.0;
t39 = d_tool_z.*t3.*t6.*t9.*1.0;
t40 = d_tool_z.*t4.*t6.*t8.*-1.0;
t41 = d_tool_z.*t4.*t6.*t8.*1.0;
t42 = d_tool_z.*t6.*t8.*t9.*-1.0;
t43 = d_tool_z.*t6.*t8.*t9.*1.0;
t44 = d_tool_y.*t8.*t9.*t10.*-1.0;
t45 = d_tool_y.*t8.*t9.*t10.*1.0;
t46 = d_tool_x.*t5.*t6.*t8.*t9;
t47 = d_tool_z.*t5.*t8.*t9.*t11;
t48 = d_tool_x.*t3.*t4.*t5.*t6.*-1.0;
t49 = d_tool_x.*t3.*t4.*t5.*t6.*1.0;
t50 = d_tool_x.*t3.*t5.*t6.*t9.*-1.0;
t51 = d_tool_x.*t3.*t5.*t6.*t9.*1.0;
t52 = d_tool_x.*t4.*t5.*t6.*t8.*-1.0;
t53 = d_tool_x.*t4.*t5.*t6.*t8.*1.0;
t54 = d_tool_z.*t3.*t4.*t5.*t11.*-1.0;
t55 = d_tool_z.*t3.*t4.*t5.*t11.*1.0;
t56 = d_tool_z.*t3.*t5.*t9.*t11.*-1.0;
t57 = d_tool_z.*t3.*t5.*t9.*t11.*1.0;
t58 = d_tool_z.*t4.*t5.*t8.*t11.*-1.0;
t59 = d_tool_z.*t4.*t5.*t8.*t11.*1.0;
t60 = t17+t23+t25+t26+t29+t33+t34+t35+t36+t42+t50+t52+t56+t58;
t61 = t13+t60;
Jt = reshape([t7.*-7.5e-2-t7.*t8.*2.7e-1+t7.*t18+t7.*t20+t7.*t22+t7.*t27+t7.*t30+t7.*t31+t7.*t32+t7.*t46+t7.*t47-d_tool_y.*t2.*t5.*1.0-d_tool_x.*t2.*t6.*t10.*1.0-d_tool_z.*t2.*t10.*t11.*1.0+d_tool_z.*t3.*t6.*t9.*t14+d_tool_z.*t4.*t6.*t8.*t14+d_tool_y.*t8.*t9.*t10.*t14+d_tool_x.*t3.*t4.*t5.*t6.*t14+d_tool_z.*t3.*t4.*t5.*t11.*t14,t2.*7.5e-2+t2.*t8.*2.7e-1+t2.*t19+t2.*t21+t2.*t28-t2.*t30.*1.0-t2.*t31.*1.0-t2.*t32.*1.0-t2.*t46.*1.0-t2.*t47.*1.0+d_tool_y.*t5.*t14-t2.*t3.*t4.*9.0e-2+d_tool_x.*t6.*t10.*t14+d_tool_z.*t10.*t11.*t14+d_tool_z.*t2.*t3.*t6.*t9+d_tool_z.*t2.*t4.*t6.*t8+d_tool_y.*t2.*t8.*t9.*t10+d_tool_x.*t2.*t3.*t4.*t5.*t6+d_tool_z.*t2.*t3.*t4.*t5.*t11,0.0,0.0,0.0,1.0,t2.*t61,t7.*t61,t8.*-2.7e-1+t18+t20+t22+t27+t30+t31+t32+t38+t40+t44+t46+t47+t48+t54,t14,t2,0.0,t2.*t60,t7.*t60,t18+t20+t22+t27+t30+t31+t32+t38+t40+t44+t46+t47+t48+t54,t14,t2,0.0,d_tool_y.*t7.*t10-d_tool_x.*t5.*t6.*t7-d_tool_z.*t5.*t7.*t11-d_tool_y.*t2.*t3.*t4.*t5+d_tool_y.*t2.*t5.*t8.*t9-d_tool_x.*t2.*t3.*t4.*t6.*t10-d_tool_z.*t2.*t3.*t4.*t10.*t11+d_tool_x.*t2.*t6.*t8.*t9.*t10+d_tool_z.*t2.*t8.*t9.*t10.*t11,d_tool_y.*t2.*t10.*-1.0+d_tool_x.*t2.*t5.*t6+d_tool_z.*t2.*t5.*t11+d_tool_y.*t3.*t4.*t5.*t14+d_tool_y.*t5.*t7.*t8.*t9+d_tool_x.*t3.*t4.*t6.*t10.*t14+d_tool_x.*t6.*t7.*t8.*t9.*t10+d_tool_z.*t3.*t4.*t10.*t11.*t14+d_tool_z.*t7.*t8.*t9.*t10.*t11,t16.*(d_tool_y.*t5+d_tool_x.*t6.*t10+d_tool_z.*t10.*t11),t2.*t16,t7.*t16,cos(t12),t2.*t5.*t29+t2.*t5.*t35-d_tool_z.*t6.*t7.*t10+d_tool_x.*t7.*t10.*t11-d_tool_x.*t2.*t3.*t6.*t9-d_tool_x.*t2.*t4.*t6.*t8-d_tool_z.*t2.*t3.*t9.*t11-d_tool_z.*t2.*t4.*t8.*t11-d_tool_x.*t2.*t3.*t4.*t5.*t11-d_tool_z.*t2.*t5.*t6.*t8.*t9,t5.*t7.*t29+t5.*t7.*t35+d_tool_z.*t2.*t6.*t10-d_tool_x.*t2.*t10.*t11-d_tool_x.*t3.*t6.*t7.*t9-d_tool_x.*t4.*t6.*t7.*t8-d_tool_z.*t3.*t7.*t9.*t11-d_tool_z.*t4.*t7.*t8.*t11-d_tool_x.*t3.*t4.*t5.*t7.*t11-d_tool_z.*t5.*t6.*t7.*t8.*t9,t5.*t31+t5.*t32-d_tool_x.*t3.*t4.*t6-d_tool_z.*t3.*t4.*t11+d_tool_x.*t6.*t8.*t9+d_tool_z.*t8.*t9.*t11-d_tool_z.*t3.*t5.*t6.*t9-d_tool_z.*t4.*t5.*t6.*t8,-t5.*t7-t2.*t3.*t4.*t10+t2.*t8.*t9.*t10,t2.*t5+t3.*t4.*t10.*t14+t7.*t8.*t9.*t10,t10.*t16],[6,5]);
