function T = codegen_dynamicsTheta(in1,in2,in3)
%CODEGEN_DYNAMICSTHETA
%    T = CODEGEN_DYNAMICSTHETA(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 13:12:21

% Regressor generated to work with the following theta matrix:
% 
%  T =
% 
% 	I_xx_5 - 1.0*I_yy_5 - 1.0*I_zz_5 - 2.0*m_5*r_cx_5^2
% 	I_xx_2 + I_xx_3 + I_xx_4 + I_yy_1 + I_zz_5 + 0.005625*m_1 + 0.15*m_1*r_cx_1 + 0.020833333333333333333333333333333*m_2*r_cx_2 - 0.295*m_3*r_cz_3 - 0.295*m_4*r_cy_4 + m_1*r_cx_1^2 + m_1*r_cz_1^2 + m_2*r_cy_2^2 + m_2*r_cz_2^2 + m_3*r_cy_3^2 + m_3*r_cz_3^2 + m_4*r_cy_4^2 + m_4*r_cz_4^2 + m_5*r_cx_5^2 + m_5*r_cy_5^2
% 	I_xx_2 - 1.0*I_yy_2 + I_zz_2 + 2.0*m_2*r_cy_2^2
% 	I_xx_4 + I_yy_4 - 1.0*I_zz_4 + 2.0*I_zz_5 + 2.0*m_4*r_cz_4^2 + 2.0*m_5*r_cx_5^2 + 2.0*m_5*r_cy_5^2
% 	I_xx_3 + 2.0*I_xx_4 + I_yy_3 - 1.0*I_zz_3 + 2.0*I_zz_5 - 0.59*m_3*r_cz_3 - 0.59*m_4*r_cy_4 + 2.0*m_3*r_cz_3^2 + 2.0*m_4*r_cy_4^2 + 2.0*m_4*r_cz_4^2 + 2.0*m_5*r_cx_5^2 + 2.0*m_5*r_cy_5^2
% 	m_2 - 3.7037037037037037037037037037037*m_2*r_cx_2 + 11.111111111111111111111111111111*m_3*r_cx_3
% 	m_3 - 11.111111111111111111111111111111*m_3*r_cx_3 - 3.3898305084745762711864406779661*m_3*r_cz_3 + 3.3898305084745762711864406779661*m_4*r_cy_4
% 	m_4 + m_5 + 3.3898305084745762711864406779661*m_3*r_cz_3 - 3.3898305084745762711864406779661*m_4*r_cy_4
% 	I_zz_2 - 0.27*m_2*r_cx_2 + m_2*r_cx_2^2 + m_2*r_cy_2^2
% 	I_xx_4 + I_yy_3 + I_zz_5 - 0.09*m_3*r_cx_3 - 0.295*m_3*r_cz_3 - 0.295*m_4*r_cy_4 + m_3*r_cx_3^2 + m_3*r_cz_3^2 + m_4*r_cy_4^2 + m_4*r_cz_4^2 + m_5*r_cx_5^2 + m_5*r_cy_5^2
% 	I_xx_4 - 1.0*I_zz_4 + I_zz_5 - 1.0*m_4*r_cx_4^2 + m_4*r_cz_4^2 + m_5*r_cx_5^2 + m_5*r_cy_5^2
% 	I_yy_5 + m_5*r_cx_5^2 + m_5*r_cz_5^2
% 	I_yz_2 - 1.0*m_2*r_cy_2*r_cz_2
% 	I_xz_2 + 0.27*m_2*r_cz_2 + 0.27*m_3*r_cy_3 - 1.0*m_2*r_cx_2*r_cz_2
% 	I_xy_2 - 1.0*m_2*r_cx_2*r_cy_2
% 	I_xz_3 - 1.0*m_3*r_cx_3*r_cz_3
% 	I_xz_5 - 1.0*m_5*r_cx_5*r_cz_5
% 	m_2*r_cy_2
% 	I_xy_3 + 0.09*m_3*r_cy_3 - 1.0*m_3*r_cx_3*r_cy_3
% 	I_xz_4 - 1.0*m_4*r_cx_4*r_cz_4
% 	I_yz_3 - 1.0*m_3*r_cy_3*r_cz_3
% 	I_xy_4 - 1.0*m_4*r_cx_4*r_cy_4
% 	I_yz_4 - 1.0*m_4*r_cy_4*r_cz_4
% 	m_4*r_cx_4
% 	m_4*r_cz_4 + m_5*r_cy_5
% 	m_5*r_cx_5
% 	I_yz_5 - 1.0*m_5*r_cy_5*r_cz_5
% 	I_xy_5 - 1.0*m_5*r_cx_5*r_cy_5
% 	m_5*r_cz_5
% 
% Robot make ID: 172193136101978c4f7794a472c8bc64
I_xx_2 = in1(2);
I_xx_3 = in1(3);
I_xy_2 = in1(17);
I_xx_4 = in1(4);
I_xy_3 = in1(18);
I_xz_2 = in1(22);
I_xx_5 = in1(5);
I_xy_4 = in1(19);
I_xz_3 = in1(23);
I_xy_5 = in1(20);
I_xz_4 = in1(24);
I_xz_5 = in1(25);
I_yy_1 = in1(6);
I_yy_2 = in1(7);
I_yy_3 = in1(8);
I_yz_2 = in1(27);
I_yy_4 = in1(9);
I_yz_3 = in1(28);
I_yy_5 = in1(10);
I_yz_4 = in1(29);
I_yz_5 = in1(30);
I_zz_2 = in1(12);
I_zz_3 = in1(13);
I_zz_4 = in1(14);
I_zz_5 = in1(15);
m_1 = in2(1,:);
m_2 = in2(2,:);
m_3 = in2(3,:);
m_4 = in2(4,:);
m_5 = in2(5,:);
r_cx_1 = in3(1);
r_cx_2 = in3(4);
r_cx_3 = in3(7);
r_cy_2 = in3(5);
r_cz_1 = in3(3);
r_cx_4 = in3(10);
r_cy_3 = in3(8);
r_cz_2 = in3(6);
r_cx_5 = in3(13);
r_cy_4 = in3(11);
r_cz_3 = in3(9);
r_cy_5 = in3(14);
r_cz_4 = in3(12);
r_cz_5 = in3(15);
t2 = r_cy_2.^2;
t3 = r_cx_5.^2;
t4 = r_cy_4.^2;
t5 = r_cz_3.^2;
t6 = r_cy_5.^2;
t7 = r_cz_4.^2;
t8 = I_zz_4.*-1.0;
t9 = I_zz_4.*1.0;
t10 = I_zz_5.*2.0;
t11 = m_3.*r_cz_3.*-2.95e-1;
t12 = m_3.*r_cz_3.*2.95e-1;
t13 = m_4.*r_cy_4.*-2.95e-1;
t14 = m_4.*r_cy_4.*2.95e-1;
t15 = m_3.*r_cx_3.*1.111111111111111e+1;
t16 = m_3.*r_cz_3.*3.389830508474576;
t17 = m_4.*r_cy_4.*3.389830508474576;
t18 = m_2.*t2;
t19 = m_3.*t5;
t20 = m_4.*t4;
t21 = m_4.*t7;
t22 = m_5.*t3;
t23 = m_5.*t6;
t24 = t21.*2.0;
t25 = t22.*2.0;
t26 = t23.*2.0;
T = [I_xx_5-I_yy_5.*1.0-I_zz_5.*1.0-t25;I_xx_2+I_xx_3+I_xx_4+I_yy_1+I_zz_5+m_1.*5.625e-3+t11+t13+t18+t19+t20+t21+t22+t23+m_1.*r_cx_1.*1.5e-1+m_2.*r_cx_2.*2.083333333333333e-2+m_1.*r_cx_1.^2+m_1.*r_cz_1.^2+m_2.*r_cz_2.^2+m_3.*r_cy_3.^2;I_xx_2-I_yy_2.*1.0+I_zz_2+t18.*2.0;I_xx_4+I_yy_4+t8+t10+t24+t25+t26;I_xx_3+I_xx_4.*2.0+I_yy_3-I_zz_3.*1.0+t10+t19.*2.0+t20.*2.0+t24+t25+t26-m_3.*r_cz_3.*5.9e-1-m_4.*r_cy_4.*5.9e-1;m_2+t15-m_2.*r_cx_2.*3.703703703703704;m_3-t15-t16+t17;m_4+m_5+t16-t17;I_zz_2+t18-m_2.*r_cx_2.*2.7e-1+m_2.*r_cx_2.^2;I_xx_4+I_yy_3+I_zz_5+t11+t13+t19+t20+t21+t22+t23-m_3.*r_cx_3.*9.0e-2+m_3.*r_cx_3.^2;I_xx_4+I_zz_5+t8+t21+t22+t23-m_4.*r_cx_4.^2.*1.0;I_yy_5+t22+m_5.*r_cz_5.^2;I_yz_2-m_2.*r_cy_2.*r_cz_2.*1.0;I_xz_2+m_2.*r_cz_2.*2.7e-1+m_3.*r_cy_3.*2.7e-1-m_2.*r_cx_2.*r_cz_2.*1.0;I_xy_2-m_2.*r_cx_2.*r_cy_2.*1.0;I_xz_3-m_3.*r_cx_3.*r_cz_3.*1.0;I_xz_5-m_5.*r_cx_5.*r_cz_5.*1.0;m_2.*r_cy_2;I_xy_3+m_3.*r_cy_3.*9.0e-2-m_3.*r_cx_3.*r_cy_3.*1.0;I_xz_4-m_4.*r_cx_4.*r_cz_4.*1.0;I_yz_3-m_3.*r_cy_3.*r_cz_3.*1.0;I_xy_4-m_4.*r_cx_4.*r_cy_4.*1.0;I_yz_4-m_4.*r_cy_4.*r_cz_4.*1.0;m_4.*r_cx_4;m_4.*r_cz_4+m_5.*r_cy_5;m_5.*r_cx_5;I_yz_5-m_5.*r_cy_5.*r_cz_5.*1.0;I_xy_5-m_5.*r_cx_5.*r_cy_5.*1.0;m_5.*r_cz_5];
