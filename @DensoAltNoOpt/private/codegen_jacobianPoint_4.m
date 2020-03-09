function Jp = codegen_jacobianPoint_4(in1,in2)
%CODEGEN_JACOBIANPOINT_4
%    JP = CODEGEN_JACOBIANPOINT_4(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 07:30:30

%Input order: Q, r
q_1 = in1(1,:);
q_2 = in1(2,:);
q_3 = in1(3,:);
q_4 = in1(4,:);
r_1 = in2(1,:);
r_2 = in2(2,:);
r_3 = in2(3,:);
t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = cos(q_4);
t6 = conj(r_1);
t7 = conj(r_2);
t8 = conj(r_3);
t9 = sin(q_1);
t10 = sin(q_2);
t11 = sin(q_3);
t12 = sin(q_4);
t13 = q_2+q_3;
t22 = -q_4;
t14 = t3.*2.7e-1;
t15 = t9.*-1.0;
t16 = t9.*1.0;
t17 = t10.*2.7e-1;
t18 = t13+1.274681180919932;
t19 = cos(t13);
t20 = q_4+t13;
t21 = sin(t13);
t31 = t13+t22;
t23 = cos(t20);
t24 = sin(t20);
t25 = cos(t18);
t26 = t19.*2.95e-1;
t27 = t21.*-2.95e-1;
t28 = t21.*2.95e-1;
t29 = t19.*9.0e-2;
t30 = t21.*9.0e-2;
t34 = t7.*t21;
t35 = t7.*t19.*-1.0;
t36 = t7.*t19.*1.0;
t37 = cos(t31);
t38 = sin(t31);
t42 = t8.*t12.*t21;
t43 = t5.*t6.*t21.*-1.0;
t44 = t5.*t6.*t21.*1.0;
t32 = t25.*-3.084234102658227e-1;
t33 = t25.*3.084234102658227e-1;
t39 = t6.*t23.*-5.0e-1;
t40 = t6.*t23.*5.0e-1;
t41 = t8.*t24.*5.0e-1;
t45 = t6.*t37.*-5.0e-1;
t46 = t6.*t37.*5.0e-1;
t47 = t8.*t38.*-5.0e-1;
t48 = t8.*t38.*5.0e-1;
t50 = t26+t30+t35+t42+t43;
t49 = t17+t32+7.5e-2;
t51 = t14+t50;
Jp = reshape([t9.*t34+t15.*t49-t6.*(t2.*t12+t3.*t4.*t5.*t9+t5.*t10.*t11.*t15).*1.0-t8.*(t2.*t5+t3.*t4.*t12.*t15+t9.*t10.*t11.*t12).*1.0,t8.*(t5.*t9+t2.*t3.*t4.*t12-t2.*t10.*t11.*t12.*1.0).*-1.0-t2.*t34.*1.0+t2.*t49-t6.*(t9.*t12-t2.*t3.*t4.*t5+t2.*t5.*t10.*t11).*1.0,0.0,0.0,0.0,1.0,t2.*t51,t9.*t51,-t17+t27+t29+t34+t39+t41+t45+t47,t15,t2,0.0,t2.*t50,t9.*t50,t27+t29+t34+t39+t41+t45+t47,t15,t2,0.0,-t5.*t6.*t9+t8.*t9.*t12-t2.*t3.*t4.*t5.*t8-t2.*t3.*t4.*t6.*t12+t2.*t5.*t8.*t10.*t11+t2.*t6.*t10.*t11.*t12,t2.*t5.*t6-t2.*t8.*t12.*1.0+t3.*t4.*t5.*t8.*t15+t3.*t4.*t6.*t12.*t15+t5.*t8.*t9.*t10.*t11+t6.*t9.*t10.*t11.*t12,t21.*(t5.*t8+t6.*t12),t2.*t21,t9.*t21,t19,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[6,6]);
