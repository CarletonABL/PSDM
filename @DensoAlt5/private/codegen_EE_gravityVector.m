function G = codegen_EE_gravityVector(in1,in2)
%CODEGEN_EE_GRAVITYVECTOR
%    G = CODEGEN_EE_GRAVITYVECTOR(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 12:11:44

%Inputs: Q, Xtool.
m_tool = in2(:,1);
q_2 = in1(2,:);
q_3 = in1(3,:);
q_4 = in1(4,:);
q_5 = in1(5,:);
r_c_tool_x = in2(:,2);
r_c_tool_y = in2(:,3);
r_c_tool_z = in2(:,4);
t2 = cos(q_2);
t3 = cos(q_3);
t4 = cos(q_4);
t5 = cos(q_5);
t6 = sin(q_2);
t7 = sin(q_3);
t8 = sin(q_4);
t9 = sin(q_5);
t10 = q_2+q_3;
t11 = cos(t10);
t12 = sin(t10);
t17 = m_tool.*r_c_tool_z.*t2.*t5.*t7.*-9.806132;
t18 = m_tool.*r_c_tool_z.*t2.*t5.*t7.*9.806132;
t19 = m_tool.*r_c_tool_z.*t3.*t5.*t6.*-9.806132;
t20 = m_tool.*r_c_tool_z.*t3.*t5.*t6.*9.806132;
t21 = m_tool.*r_c_tool_x.*t2.*t7.*t9.*9.806132;
t22 = m_tool.*r_c_tool_x.*t3.*t6.*t9.*9.806132;
t23 = m_tool.*r_c_tool_x.*t4.*t5.*t6.*t7.*9.806132;
t24 = m_tool.*r_c_tool_z.*t4.*t6.*t7.*t9.*9.806132;
t25 = m_tool.*r_c_tool_x.*t2.*t3.*t4.*t5.*-9.806132;
t26 = m_tool.*r_c_tool_x.*t2.*t3.*t4.*t5.*9.806132;
t27 = m_tool.*r_c_tool_z.*t2.*t3.*t4.*t9.*-9.806132;
t28 = m_tool.*r_c_tool_z.*t2.*t3.*t4.*t9.*9.806132;
t13 = m_tool.*t12.*-2.89280894;
t14 = m_tool.*t12.*2.89280894;
t15 = m_tool.*t11.*8.8255188e-1;
t16 = m_tool.*r_c_tool_y.*t8.*t11.*9.806132;
G = [0.0;t13+t15+t16+t17+t19+t21+t22+t23+t24+t25+t27-m_tool.*t6.*2.64765564;t13+t15+t16+t17+t19+t21+t22+t23+t24+t25+t27;m_tool.*r_c_tool_y.*t4.*t12.*9.806132+m_tool.*r_c_tool_x.*t2.*t5.*t7.*t8.*9.806132+m_tool.*r_c_tool_x.*t3.*t5.*t6.*t8.*9.806132+m_tool.*r_c_tool_z.*t2.*t7.*t8.*t9.*9.806132+m_tool.*r_c_tool_z.*t3.*t6.*t8.*t9.*9.806132;t4.*t17+t4.*t19+t4.*t21+t4.*t22-m_tool.*r_c_tool_x.*t2.*t3.*t5.*9.806132-m_tool.*r_c_tool_z.*t2.*t3.*t9.*9.806132+m_tool.*r_c_tool_x.*t5.*t6.*t7.*9.806132+m_tool.*r_c_tool_z.*t6.*t7.*t9.*9.806132];
