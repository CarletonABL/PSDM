function D = codegen_EE_massMatrix(in1,in2)
%CODEGEN_EE_MASSMATRIX
%    D = CODEGEN_EE_MASSMATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    03-Feb-2020 09:41:33

%Inputs: Q, Xtool.
I_xx_tool = in2(:,5);
I_xy_tool = in2(:,8);
I_xz_tool = in2(:,9);
I_yy_tool = in2(:,6);
I_yz_tool = in2(:,10);
I_zz_tool = in2(:,7);
m_tool = in2(:,1);
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
q_4 = in1(4,:);
r_c_tool_x = in2(:,2);
r_c_tool_y = in2(:,3);
r_c_tool_z = in2(:,4);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = cos(q_4);
t6 = sin(q_1);
t7 = sin(q_2);
t8 = sin(q_3);
t9 = sin(q_4);
t10 = q_4.*2.0;
t11 = m_tool.*9.5125e-2;
t12 = q_2+q_3;
t13 = r_c_tool_x.^2;
t14 = r_c_tool_y.^2;
t15 = r_c_tool_z.^2;
t20 = m_tool.*r_c_tool_y.*-5.9e-1;
t21 = m_tool.*r_c_tool_y.*5.9e-1;
t16 = t5.^2;
t17 = cos(t10);
t18 = sin(t10);
t19 = t7.*2.7e-1;
t22 = t12+1.274681180919932;
t23 = cos(t12);
t24 = sin(t12);
t25 = m_tool.*t8.*2.43e-2;
t26 = m_tool.*t14;
t27 = m_tool.*t15;
t28 = I_yz_tool.*t5.*-1.0;
t29 = I_yz_tool.*t5.*1.0;
t30 = I_xy_tool.*t9.*-1.0;
t31 = I_xy_tool.*t9.*1.0;
t32 = m_tool.*t4.*7.965e-2;
t35 = I_yy_tool.*t3.*t4;
t36 = m_tool.*r_c_tool_y.*t4.*-2.7e-1;
t37 = m_tool.*r_c_tool_y.*t4.*2.7e-1;
t38 = m_tool.*r_c_tool_z.*t5.*-2.95e-1;
t39 = m_tool.*r_c_tool_z.*t5.*2.95e-1;
t40 = m_tool.*r_c_tool_x.*t5.*7.5e-2;
t41 = m_tool.*r_c_tool_x.*t9.*-2.95e-1;
t42 = m_tool.*r_c_tool_x.*t9.*2.95e-1;
t43 = m_tool.*r_c_tool_x.*t5.*-1.8e-1;
t44 = m_tool.*r_c_tool_x.*t5.*1.8e-1;
t45 = m_tool.*r_c_tool_z.*t9.*-7.5e-2;
t46 = m_tool.*r_c_tool_z.*t9.*7.5e-2;
t47 = m_tool.*r_c_tool_z.*t9.*1.8e-1;
t48 = m_tool.*r_c_tool_y.*r_c_tool_z.*t5;
t49 = m_tool.*r_c_tool_x.*r_c_tool_y.*t9;
t55 = I_yy_tool.*t7.*t8.*-1.0;
t56 = I_yy_tool.*t7.*t8.*1.0;
t57 = m_tool.*r_c_tool_z.*t3.*t5.*-2.7e-1;
t58 = m_tool.*r_c_tool_z.*t3.*t5.*2.7e-1;
t59 = m_tool.*r_c_tool_z.*t4.*t5.*-2.7e-1;
t60 = m_tool.*r_c_tool_z.*t4.*t5.*2.7e-1;
t61 = I_xy_tool.*t3.*t5.*t8;
t62 = I_xy_tool.*t4.*t5.*t7;
t63 = m_tool.*r_c_tool_x.*t3.*t9.*-2.7e-1;
t64 = m_tool.*r_c_tool_x.*t3.*t9.*2.7e-1;
t66 = m_tool.*r_c_tool_x.*t4.*t9.*-2.7e-1;
t67 = m_tool.*r_c_tool_x.*t4.*t9.*2.7e-1;
t68 = m_tool.*r_c_tool_x.*t5.*t8.*-2.7e-1;
t69 = m_tool.*r_c_tool_x.*t5.*t8.*2.7e-1;
t70 = I_yz_tool.*t5.*t7.*t8;
t71 = m_tool.*r_c_tool_z.*t7.*t9.*-2.7e-1;
t73 = m_tool.*r_c_tool_z.*t8.*t9.*2.7e-1;
t74 = I_xy_tool.*t7.*t8.*t9;
t75 = m_tool.*t3.*t4.*t13;
t83 = I_yz_tool.*t3.*t8.*t9.*-1.0;
t84 = I_yz_tool.*t3.*t8.*t9.*1.0;
t85 = I_yz_tool.*t4.*t7.*t9.*-1.0;
t86 = I_yz_tool.*t4.*t7.*t9.*1.0;
t94 = m_tool.*r_c_tool_x.*t3.*t5.*t8.*2.95e-1;
t95 = m_tool.*r_c_tool_x.*t4.*t5.*t7.*2.95e-1;
t96 = m_tool.*r_c_tool_x.*t3.*t4.*t5.*-9.0e-2;
t97 = m_tool.*r_c_tool_x.*t3.*t4.*t5.*9.0e-2;
t98 = m_tool.*r_c_tool_z.*t3.*t8.*t9.*-2.95e-1;
t99 = m_tool.*r_c_tool_z.*t3.*t8.*t9.*2.95e-1;
t100 = m_tool.*r_c_tool_z.*t4.*t7.*t9.*-2.95e-1;
t101 = m_tool.*r_c_tool_z.*t4.*t7.*t9.*2.95e-1;
t103 = m_tool.*r_c_tool_z.*t3.*t4.*t9.*9.0e-2;
t104 = m_tool.*r_c_tool_z.*t3.*t5.*t8.*-9.0e-2;
t105 = m_tool.*r_c_tool_z.*t3.*t5.*t8.*9.0e-2;
t106 = m_tool.*r_c_tool_z.*t4.*t5.*t7.*-9.0e-2;
t107 = m_tool.*r_c_tool_z.*t4.*t5.*t7.*9.0e-2;
t110 = m_tool.*r_c_tool_x.*t3.*t8.*t9.*-9.0e-2;
t111 = m_tool.*r_c_tool_x.*t3.*t8.*t9.*9.0e-2;
t112 = m_tool.*r_c_tool_x.*t4.*t7.*t9.*-9.0e-2;
t113 = m_tool.*r_c_tool_x.*t4.*t7.*t9.*9.0e-2;
t114 = m_tool.*r_c_tool_x.*t5.*t7.*t8.*9.0e-2;
t116 = m_tool.*r_c_tool_z.*t7.*t8.*t9.*-9.0e-2;
t117 = m_tool.*r_c_tool_z.*t7.*t8.*t9.*9.0e-2;
t118 = m_tool.*r_c_tool_y.*r_c_tool_z.*t3.*t8.*t9;
t119 = m_tool.*r_c_tool_y.*r_c_tool_z.*t4.*t7.*t9;
t120 = m_tool.*t7.*t8.*t13.*-1.0;
t121 = m_tool.*t7.*t8.*t13.*1.0;
t134 = m_tool.*r_c_tool_x.*r_c_tool_y.*t3.*t5.*t8.*-1.0;
t135 = m_tool.*r_c_tool_x.*r_c_tool_y.*t3.*t5.*t8.*1.0;
t136 = m_tool.*r_c_tool_x.*r_c_tool_y.*t4.*t5.*t7.*-1.0;
t137 = m_tool.*r_c_tool_x.*r_c_tool_y.*t4.*t5.*t7.*1.0;
t33 = cos(t22);
t34 = I_xz_tool.*t18;
t50 = I_zz_tool.*t16;
t51 = I_xx_tool.*t16.*-1.0;
t52 = I_xx_tool.*t16.*1.0;
t65 = m_tool.*r_c_tool_x.*t5.*t19;
t72 = m_tool.*r_c_tool_z.*t9.*t19;
t76 = t3.*t4.*t27;
t77 = t3.*t4.*t28;
t78 = t3.*t4.*t29;
t79 = m_tool.*r_c_tool_x.*r_c_tool_z.*t18.*-1.0;
t80 = m_tool.*r_c_tool_x.*r_c_tool_z.*t18.*1.0;
t81 = t3.*t4.*t30;
t82 = t3.*t4.*t31;
t87 = m_tool.*t13.*t16;
t88 = t3.*t4.*t38;
t89 = t3.*t4.*t39;
t90 = t16.*t27.*-1.0;
t91 = t16.*t27.*1.0;
t92 = t3.*t4.*t41;
t93 = t3.*t4.*t42;
t102 = t7.*t8.*t39;
t108 = t3.*t4.*t48;
t109 = t7.*t8.*t42;
t115 = t3.*t4.*t49;
t122 = t7.*t8.*t27.*-1.0;
t123 = t7.*t8.*t27.*1.0;
t124 = I_xz_tool.*t3.*t8.*t17.*-1.0;
t125 = I_xz_tool.*t3.*t8.*t17.*1.0;
t126 = I_xz_tool.*t4.*t7.*t17.*-1.0;
t127 = I_xz_tool.*t4.*t7.*t17.*1.0;
t128 = I_xx_tool.*t3.*t8.*t18.*-5.0e-1;
t129 = I_xx_tool.*t3.*t8.*t18.*5.0e-1;
t130 = I_xx_tool.*t4.*t7.*t18.*-5.0e-1;
t131 = I_xx_tool.*t4.*t7.*t18.*5.0e-1;
t132 = I_zz_tool.*t3.*t8.*t18.*5.0e-1;
t133 = I_zz_tool.*t4.*t7.*t18.*5.0e-1;
t138 = t7.*t8.*t48.*-1.0;
t139 = t7.*t8.*t48.*1.0;
t140 = t7.*t8.*t49.*-1.0;
t141 = t7.*t8.*t49.*1.0;
t142 = m_tool.*r_c_tool_x.*r_c_tool_z.*t3.*t8.*t17;
t143 = m_tool.*r_c_tool_x.*r_c_tool_z.*t4.*t7.*t17;
t145 = m_tool.*t3.*t8.*t13.*t18.*5.0e-1;
t146 = m_tool.*t4.*t7.*t13.*t18.*5.0e-1;
t147 = t3.*t8.*t18.*t27.*-5.0e-1;
t148 = t3.*t8.*t18.*t27.*5.0e-1;
t149 = t4.*t7.*t18.*t27.*-5.0e-1;
t150 = t4.*t7.*t18.*t27.*5.0e-1;
t151 = t28+t30+t38+t41+t48+t49;
t53 = t33.*-3.084234102658227e-1;
t54 = t33.*3.084234102658227e-1;
t152 = t59+t66+t151;
t153 = I_xx_tool+t11+t20+t25+t26+t27+t32+t34+t36+t43+t47+t50+t51+t68+t73+t79+t87+t90;
t154 = t35+t40+t45+t55+t61+t62+t65+t71+t75+t76+t83+t85+t94+t95+t96+t98+t100+t103+t114+t116+t118+t119+t120+t122+t134+t136;
t155 = t70+t74+t77+t81+t88+t92+t102+t104+t106+t108+t109+t110+t112+t115+t124+t126+t128+t130+t132+t133+t138+t140+t142+t143+t145+t146+t147+t149;
t144 = t19+t53+7.5e-2;
t156 = t57+t63+t155;
D = reshape([m_tool.*((r_c_tool_x.*(t2.*t9+t3.*t4.*t5.*t6-t5.*t6.*t7.*t8.*1.0)+r_c_tool_z.*(t2.*t5-t3.*t4.*t6.*t9.*1.0+t6.*t7.*t8.*t9)+t6.*t144-r_c_tool_y.*t6.*t24.*1.0).^2+(r_c_tool_z.*(t5.*t6+t2.*t3.*t4.*t9-t2.*t7.*t8.*t9.*1.0)-t2.*t144.*1.0+r_c_tool_x.*(t6.*t9-t2.*t3.*t4.*t5+t2.*t5.*t7.*t8)+r_c_tool_y.*t2.*t24).^2)+t23.*(I_yy_tool.*t23+I_xy_tool.*t5.*t24-I_yz_tool.*t9.*t24.*1.0)+t5.*t24.*(I_xy_tool.*t23+I_xx_tool.*t5.*t24-I_xz_tool.*t9.*t24.*1.0)-t9.*t24.*(I_yz_tool.*t23+I_xz_tool.*t5.*t24-I_zz_tool.*t9.*t24.*1.0).*1.0,t156,t155,t154,t156,I_xx_tool+m_tool.*1.68025e-1+t20+t26+t27+t34+t43+t47+t50+t51+t79+t87+t90+m_tool.*t4.*1.593e-1+m_tool.*t8.*4.86e-2-m_tool.*r_c_tool_y.*t4.*5.4e-1-m_tool.*r_c_tool_x.*t5.*t8.*5.4e-1+m_tool.*r_c_tool_z.*t8.*t9.*5.4e-1,t153,t152,t155,t153,I_xx_tool+t11+t20+t26+t27+t34+t43+t47+t50+t51+t79+t87+t90,t151,t154,t152,t151,I_yy_tool+t27+m_tool.*t13],[4,4]);
