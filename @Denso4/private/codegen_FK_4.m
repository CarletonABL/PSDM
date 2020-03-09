function T = codegen_FK_4(in1)
%CODEGEN_FK_4
%    T = CODEGEN_FK_4(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    11-Feb-2020 15:36:03

%Input order: Q.
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
t11 = t7.*2.7e-1;
t12 = t10+1.274681180919932;
t13 = sin(t10);
t14 = cos(t12);
t15 = t14.*-3.084234102658227e-1;
t16 = t14.*3.084234102658227e-1;
t17 = t11+t15+7.5e-2;
T = reshape([t6.*t9.*-1.0+t2.*t3.*t4.*t5-t2.*t5.*t7.*t8.*1.0,t2.*t9+t3.*t4.*t5.*t6-t5.*t6.*t7.*t8.*1.0,t5.*t13.*-1.0,0.0,t2.*t13.*-1.0,t6.*t13.*-1.0,cos(t10).*-1.0,0.0,-t5.*t6-t2.*t3.*t4.*t9+t2.*t7.*t8.*t9,t2.*t5-t3.*t4.*t6.*t9.*1.0+t6.*t7.*t8.*t9,t9.*t13,0.0,t2.*t17,t6.*t17,t3.*2.7e-1+cos(t10-2.961151458749647e-1).*3.084234102658227e-1+1.32e-1,1.0],[4,4]);
