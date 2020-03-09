function J = codegen_jacobian_5(in1)
%CODEGEN_JACOBIAN_5
%    J = CODEGEN_JACOBIAN_5(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 07:30:02

%Input order: Q
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
q_4 = in1(4,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = cos(q_4);
t6 = sin(q_1);
t7 = sin(q_2);
t8 = sin(q_3);
t9 = sin(q_4);
t10 = q_2+q_3;
t11 = t3.*2.7e-1;
t12 = t6.*-1.0;
t13 = t6.*1.0;
t14 = t7.*2.7e-1;
t15 = t10-2.961151458749647e-1;
t16 = t10+1.274681180919932;
t17 = sin(t10);
t18 = cos(t15);
t19 = cos(t16);
t20 = t18.*3.084234102658227e-1;
t21 = t19.*-3.084234102658227e-1;
t22 = t19.*3.084234102658227e-1;
t23 = t11+t20;
t24 = t14+t21+7.5e-2;
J = reshape([t12.*t24,t2.*t24,0.0,0.0,0.0,1.0,t2.*t23,t6.*t23,-t14+t22,t12,t2,0.0,t2.*t18.*3.084234102658227e-1,t6.*t18.*3.084234102658227e-1,t19.*3.084234102658227e-1,t12,t2,0.0,0.0,0.0,0.0,t2.*t17,t6.*t17,cos(t10),0.0,0.0,0.0,-t5.*t6-t2.*t3.*t4.*t9+t2.*t7.*t8.*t9,t2.*t5+t3.*t4.*t9.*t12+t6.*t7.*t8.*t9,t9.*t17,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);
