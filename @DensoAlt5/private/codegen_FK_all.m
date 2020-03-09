function T = codegen_FK_all(in1,in2)
%CODEGEN_FK_ALL
%    T = CODEGEN_FK_ALL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 11:33:49

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
t12 = q_2+q_3;
t13 = t3.*2.7e-1;
t14 = t2.*7.5e-2;
t15 = t7.*-1.0;
t16 = t7.*1.0;
t17 = t8.*2.7e-1;
t18 = t12-2.961151458749647e-1;
t19 = t7.*7.5e-2;
t20 = t12+1.274681180919932;
t21 = cos(t12);
t22 = sin(t12);
t23 = t2.*t5;
t24 = t5.*t7;
t34 = t2.*t6.*t10;
t36 = t2.*t10.*t11;
t37 = t6.*t7.*t10;
t40 = t2.*t3.*t4.*t10;
t41 = t2.*t3.*t6.*t9;
t42 = t2.*t4.*t6.*t8;
t43 = t2.*t3.*t9.*t11;
t44 = t2.*t4.*t8.*t11;
t45 = t3.*t6.*t7.*t9;
t46 = t4.*t6.*t7.*t8;
t47 = t2.*t8.*t9.*t10;
t48 = t3.*t7.*t9.*t11;
t49 = t4.*t7.*t8.*t11;
t51 = t7.*t8.*t9.*t10;
t25 = cos(t18);
t26 = cos(t20);
t27 = t17+7.5e-2;
t28 = t21.*-9.0e-2;
t29 = t21.*9.0e-2;
t30 = -t24;
t35 = t10.*t22;
t38 = t10.*t11.*t15;
t39 = t10.*t11.*t16;
t50 = -t34;
t52 = t3.*t4.*t10.*t15;
t53 = t3.*t4.*t10.*t16;
t54 = t47.*-1.0;
t55 = t47.*1.0;
t56 = t3.*t4.*t6.*t23;
t57 = t3.*t4.*t11.*t23;
t58 = t3.*t4.*t6.*t24;
t59 = t6.*t8.*t9.*t23;
t60 = t3.*t4.*t11.*t24;
t61 = t6.*t8.*t9.*t24;
t62 = -t40;
t63 = t5.*t8.*t9.*t11.*t15;
t64 = t5.*t8.*t9.*t11.*t16;
t66 = t8.*t9.*t11.*t23.*-1.0;
t67 = t8.*t9.*t11.*t23.*1.0;
t31 = t25.*3.084234102658227e-1;
t32 = t26.*-3.084234102658227e-1;
t33 = t26.*3.084234102658227e-1;
t65 = t27+t28;
t68 = -t56;
t69 = t3.*t4.*t6.*t30;
t74 = t24+t40+t54;
t75 = t23+t51+t52;
t76 = t30+t47+t62;
t77 = t36+t45+t46+t60+t63;
t79 = t38+t41+t42+t57+t66;
t70 = t13+t31+1.32e-1;
t71 = t27+t32;
t72 = t2.*(t27-t33);
t73 = t7.*(t27-t33);
t78 = t37+t43+t44+t59+t68;
t80 = t48+t49+t50+t61+t69;
T = reshape([t2,t7,0.0,0.0,0.0,0.0,-1.0,0.0,t15,t2,0.0,0.0,t14,t19,1.32e-1,1.0,t2.*t8.*-1.0,t8.*t15,t3.*-1.0,0.0,t2.*t3.*-1.0,t3.*t15,t8,0.0,t15,t2,0.0,0.0,t2.*t27,t7.*t27,t13+1.32e-1,1.0,t2.*t21,t7.*t21,t22.*-1.0,0.0,t15,t2,0.0,0.0,t2.*t22,t7.*t22,t21,0.0,t2.*(t27-t29),t7.*(t27-t29),t13+t22.*9.0e-2+1.32e-1,1.0,t10.*t15+t3.*t4.*t23-t8.*t9.*t23.*1.0,t2.*t10+t3.*t4.*t24+t5.*t8.*t9.*t15,t5.*t22.*-1.0,0.0,t2.*t22.*-1.0,t15.*t22,t21.*-1.0,0.0,t76,t75,t35,0.0,t72,t73,t70,1.0,t43.*-1.0-t44.*1.0+t56-t59.*1.0+t6.*t10.*t15,t34+t58+t3.*t9.*t11.*t15+t4.*t8.*t11.*t15+t5.*t6.*t8.*t9.*t15,-t3.*t4.*t11+t8.*t9.*t11-t3.*t5.*t6.*t9-t4.*t5.*t6.*t8,0.0,t76,t75,t35,0.0,t79,t77,t3.*t4.*t6-t6.*t8.*t9.*1.0-t3.*t5.*t9.*t11.*1.0-t4.*t5.*t8.*t11.*1.0,0.0,t72,t73,t70,1.0,r_11_tool.*t78.*-1.0-r_21_tool.*t74.*1.0+r_31_tool.*t79,r_11_tool.*t80.*-1.0+r_21_tool.*t75+r_31_tool.*t77,-r_11_tool.*t3.*t4.*t11+r_11_tool.*t8.*t9.*t11+r_21_tool.*t3.*t9.*t10+r_21_tool.*t4.*t8.*t10+r_31_tool.*t3.*t4.*t6-r_31_tool.*t6.*t8.*t9-r_11_tool.*t3.*t5.*t6.*t9-r_11_tool.*t4.*t5.*t6.*t8-r_31_tool.*t3.*t5.*t9.*t11-r_31_tool.*t4.*t5.*t8.*t11,0.0,r_12_tool.*t78.*-1.0-r_22_tool.*t74.*1.0+r_32_tool.*t79,r_12_tool.*t80.*-1.0+r_22_tool.*t75+r_32_tool.*t77,-r_12_tool.*t3.*t4.*t11+r_12_tool.*t8.*t9.*t11+r_22_tool.*t3.*t9.*t10+r_22_tool.*t4.*t8.*t10+r_32_tool.*t3.*t4.*t6-r_32_tool.*t6.*t8.*t9-r_12_tool.*t3.*t5.*t6.*t9-r_12_tool.*t4.*t5.*t6.*t8-r_32_tool.*t3.*t5.*t9.*t11-r_32_tool.*t4.*t5.*t8.*t11,0.0,r_13_tool.*t78.*-1.0-r_23_tool.*t74.*1.0+r_33_tool.*t79,r_13_tool.*t80.*-1.0+r_23_tool.*t75+r_33_tool.*t77,-r_13_tool.*t3.*t4.*t11+r_13_tool.*t8.*t9.*t11+r_23_tool.*t3.*t9.*t10+r_23_tool.*t4.*t8.*t10+r_33_tool.*t3.*t4.*t6-r_33_tool.*t6.*t8.*t9-r_13_tool.*t3.*t5.*t6.*t9-r_13_tool.*t4.*t5.*t6.*t8-r_33_tool.*t3.*t5.*t9.*t11-r_33_tool.*t4.*t5.*t8.*t11,0.0,t14+d_tool_z.*t38-d_tool_y.*t40.*1.0-d_tool_x.*t43.*1.0+d_tool_z.*t41-d_tool_x.*t44.*1.0+d_tool_z.*t42+d_tool_y.*t47+d_tool_x.*t56-d_tool_x.*t59.*1.0+d_tool_z.*t57+d_tool_z.*t66+t2.*t17+d_tool_y.*t5.*t15-t2.*t3.*t4.*9.0e-2+t2.*t3.*t9.*2.95e-1+t2.*t4.*t8.*2.95e-1+t2.*t8.*t9.*9.0e-2+d_tool_x.*t6.*t10.*t15,t19+d_tool_y.*t23+d_tool_x.*t34+d_tool_z.*t36+d_tool_z.*t45+d_tool_z.*t46+d_tool_y.*t51+d_tool_y.*t52+d_tool_x.*t58+d_tool_z.*t60+d_tool_z.*t63+t7.*t17-t3.*t4.*t7.*9.0e-2+t3.*t7.*t9.*2.95e-1+t4.*t7.*t8.*2.95e-1+t7.*t8.*t9.*9.0e-2+d_tool_x.*t3.*t9.*t11.*t15+d_tool_x.*t4.*t8.*t11.*t15+d_tool_x.*t5.*t6.*t8.*t9.*t15,t13+t3.*t4.*2.95e-1+t3.*t9.*9.0e-2+t4.*t8.*9.0e-2-t8.*t9.*2.95e-1+d_tool_z.*t3.*t4.*t6-d_tool_x.*t3.*t4.*t11.*1.0+d_tool_y.*t3.*t9.*t10+d_tool_y.*t4.*t8.*t10-d_tool_z.*t6.*t8.*t9.*1.0+d_tool_x.*t8.*t9.*t11-d_tool_x.*t3.*t5.*t6.*t9.*1.0-d_tool_x.*t4.*t5.*t6.*t8.*1.0-d_tool_z.*t3.*t5.*t9.*t11.*1.0-d_tool_z.*t4.*t5.*t8.*t11.*1.0+1.32e-1,1.0],[4,4,6]);
