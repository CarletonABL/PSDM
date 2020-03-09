function T = codegen_FK_tool(in1,in2)
%CODEGEN_FK_TOOL
%    T = CODEGEN_FK_TOOL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 13:14:35

%Input order: Q, Ttool.
d_tool_x = in2(10);
d_tool_y = in2(11);
d_tool_z = in2(12);
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
q_4 = in1(4,:);
q_5 = in1(5,:);
r_11_tool = in2(1);
r_12_tool = in2(4);
r_13_tool = in2(7);
r_21_tool = in2(2);
r_22_tool = in2(5);
r_23_tool = in2(8);
r_31_tool = in2(3);
r_32_tool = in2(6);
r_33_tool = in2(9);
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
t12 = t2.*t5;
t13 = t5.*t7;
t14 = t2.*t6.*t10;
t15 = t2.*t10.*t11;
t16 = t6.*t7.*t10;
t17 = t7.*t10.*t11.*-1.0;
t18 = t7.*t10.*t11.*1.0;
t19 = t2.*t3.*t4.*t10;
t20 = t2.*t3.*t6.*t9;
t21 = t2.*t4.*t6.*t8;
t22 = t2.*t3.*t9.*t11;
t23 = t2.*t4.*t8.*t11;
t24 = t3.*t6.*t7.*t9;
t25 = t4.*t6.*t7.*t8;
t26 = t3.*t7.*t9.*t11;
t27 = t4.*t7.*t8.*t11;
t29 = t7.*t8.*t9.*t10;
t30 = t3.*t4.*t7.*t10.*-1.0;
t31 = t3.*t4.*t7.*t10.*1.0;
t32 = t2.*t8.*t9.*t10.*-1.0;
t33 = t2.*t8.*t9.*t10.*1.0;
t28 = -t14;
t34 = t3.*t4.*t6.*t12;
t35 = t3.*t4.*t11.*t12;
t36 = t3.*t4.*t6.*t13;
t37 = t6.*t8.*t9.*t12;
t38 = t3.*t4.*t11.*t13;
t39 = t6.*t8.*t9.*t13;
t40 = t8.*t9.*t11.*t13.*-1.0;
t41 = t8.*t9.*t11.*t13.*1.0;
t42 = t8.*t9.*t11.*t12.*-1.0;
t43 = t8.*t9.*t11.*t12.*1.0;
t46 = t13+t19+t32;
t47 = t12+t29+t30;
t44 = -t34;
t45 = -t36;
t48 = t15+t24+t25+t38+t40;
t50 = t17+t20+t21+t35+t42;
t49 = t16+t22+t23+t37+t44;
t51 = t26+t27+t28+t39+t45;
T = reshape([r_11_tool.*t49.*-1.0-r_21_tool.*t46.*1.0+r_31_tool.*t50,r_11_tool.*t51.*-1.0+r_21_tool.*t47+r_31_tool.*t48,-r_11_tool.*t3.*t4.*t11+r_11_tool.*t8.*t9.*t11+r_21_tool.*t3.*t9.*t10+r_21_tool.*t4.*t8.*t10+r_31_tool.*t3.*t4.*t6-r_31_tool.*t6.*t8.*t9-r_11_tool.*t3.*t5.*t6.*t9-r_11_tool.*t4.*t5.*t6.*t8-r_31_tool.*t3.*t5.*t9.*t11-r_31_tool.*t4.*t5.*t8.*t11,0.0,r_12_tool.*t49.*-1.0-r_22_tool.*t46.*1.0+r_32_tool.*t50,r_12_tool.*t51.*-1.0+r_22_tool.*t47+r_32_tool.*t48,-r_12_tool.*t3.*t4.*t11+r_12_tool.*t8.*t9.*t11+r_22_tool.*t3.*t9.*t10+r_22_tool.*t4.*t8.*t10+r_32_tool.*t3.*t4.*t6-r_32_tool.*t6.*t8.*t9-r_12_tool.*t3.*t5.*t6.*t9-r_12_tool.*t4.*t5.*t6.*t8-r_32_tool.*t3.*t5.*t9.*t11-r_32_tool.*t4.*t5.*t8.*t11,0.0,r_13_tool.*t49.*-1.0-r_23_tool.*t46.*1.0+r_33_tool.*t50,r_13_tool.*t51.*-1.0+r_23_tool.*t47+r_33_tool.*t48,-r_13_tool.*t3.*t4.*t11+r_13_tool.*t8.*t9.*t11+r_23_tool.*t3.*t9.*t10+r_23_tool.*t4.*t8.*t10+r_33_tool.*t3.*t4.*t6-r_33_tool.*t6.*t8.*t9-r_13_tool.*t3.*t5.*t6.*t9-r_13_tool.*t4.*t5.*t6.*t8-r_33_tool.*t3.*t5.*t9.*t11-r_33_tool.*t4.*t5.*t8.*t11,0.0,t2.*7.5e-2-d_tool_y.*t13.*1.0-d_tool_x.*t16.*1.0+d_tool_z.*t17-d_tool_y.*t19.*1.0-d_tool_x.*t22.*1.0+d_tool_z.*t20-d_tool_x.*t23.*1.0+d_tool_z.*t21+d_tool_x.*t34-d_tool_x.*t37.*1.0+d_tool_z.*t35+d_tool_z.*t42+t2.*t8.*2.7e-1-t2.*t3.*t4.*9.0e-2+t2.*t3.*t9.*2.95e-1+t2.*t4.*t8.*2.95e-1+t2.*t8.*t9.*9.0e-2+d_tool_y.*t2.*t8.*t9.*t10,t7.*7.5e-2+d_tool_y.*t12+d_tool_x.*t14+d_tool_z.*t15-d_tool_x.*t26.*1.0+d_tool_z.*t24-d_tool_x.*t27.*1.0+d_tool_z.*t25+d_tool_y.*t29+d_tool_y.*t30+d_tool_x.*t36-d_tool_x.*t39.*1.0+d_tool_z.*t38+d_tool_z.*t40+t7.*t8.*2.7e-1-t3.*t4.*t7.*9.0e-2+t3.*t7.*t9.*2.95e-1+t4.*t7.*t8.*2.95e-1+t7.*t8.*t9.*9.0e-2,t3.*2.7e-1+t3.*t4.*2.95e-1+t3.*t9.*9.0e-2+t4.*t8.*9.0e-2-t8.*t9.*2.95e-1+d_tool_z.*t3.*t4.*t6-d_tool_x.*t3.*t4.*t11.*1.0+d_tool_y.*t3.*t9.*t10+d_tool_y.*t4.*t8.*t10-d_tool_z.*t6.*t8.*t9.*1.0+d_tool_x.*t8.*t9.*t11-d_tool_x.*t3.*t5.*t6.*t9.*1.0-d_tool_x.*t4.*t5.*t6.*t8.*1.0-d_tool_z.*t3.*t5.*t9.*t11.*1.0-d_tool_z.*t4.*t5.*t8.*t11.*1.0+1.32e-1,1.0],[4,4]);
