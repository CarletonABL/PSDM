  /*
  Inputs: Q, Qd.
  */
  t2 = cos(q_3);
  t3 = cos(q_4);
  t4 = sin(q_4);
  t5 = q_4*-2.0;
  t6 = q_4*2.0;
  t7 = q_4+1.227772386374193;
  t8 = q_4+1.227772386374193;
  t9 = q_2+9.313477262058242E-2;
  t10 = q_3-7.755911012277234E-1;
  t11 = q_4-7.831785454340064E-1;
  t12 = q_2+q_3;
  t13 = q_2*2.0;
  t14 = q_3*2.0;
  t15 = q_4*2.0;
  t21 = q_2+q_4-3.430239404207034E-1;
  t22 = q_3+q_4-3.430239404207034E-1;
  t27 = -q_4;
  t16 = cos(t7);
  t17 = cos(t8);
  t18 = cos(t9);
  t19 = cos(t10);
  t20 = cos(t11);
  t23 = t3*1.566348E-3;
  t24 = cos(t12);
  t25 = t12+1.567476836912762;
  t26 = t4*1.55941E-3;
  t29 = t15-1.068495677443165E-2;
  t30 = q_4+t12-1.052751428494702;
  t31 = q_4+t12-1.052751428494702;
  t32 = cos(t21);
  t33 = cos(t22);
  t40 = t2*t3*2.1924E-3;
  t45 = t2*t4*7.83E-4;
  t50 = t5+t12+1.068495677443165E-2;
  t51 = t6+t12-1.068495677443165E-2;
  t54 = q_2+t27+3.430239404207034E-1;
  t55 = q_3+t27+3.430239404207034E-1;
  t63 = t12+t27+4.257188103684256E-1;
  t28 = cos(t25);
  t34 = t16*-6.46673990508355E-4;
  t35 = t16*6.46673990508355E-4;
  t36 = t20*2.210250122769819E-3;
  t37 = t17*1.552017577220052E-3;
  t38 = t18*-2.250455669747263E-2;
  t39 = t18*2.250455669747263E-2;
  t41 = cos(t30);
  t42 = cos(t31);
  t43 = t24*5.825636E-3;
  t44 = t19*2.221275589396552E-1;
  t46 = cos(t29);
  t47 = t32*-1.164013182915039E-3;
  t48 = t32*1.164013182915039E-3;
  t49 = t33*1.164013182915039E-3;
  t56 = cos(t50);
  t57 = cos(t51);
  t61 = cos(t54);
  t62 = cos(t55);
  t66 = cos(t63);
  t74 = t23+t26+t40+t45;
  t52 = t28*-1.205006638985861E-3;
  t53 = t28*1.205006638985861E-3;
  t58 = t41*1.318047347898018E-3;
  t59 = t42*-1.318047347898018E-3;
  t60 = t42*1.318047347898018E-3;
  t64 = t46*-7.599613813372361E-4;
  t65 = t46*7.599613813372361E-4;
  t67 = t61*1.164013182915039E-3;
  t68 = t62*-1.164013182915039E-3;
  t69 = t62*1.164013182915039E-3;
  t70 = t56*-3.799806906686181E-4;
  t71 = t56*3.799806906686181E-4;
  t72 = t57*3.799806906686181E-4;
  t73 = t66*1.003219223949083E-3;
  t75 = t37+t44+t49+t64+t68+1.96250007E-1;
  t76 = t52+t58+t70+t72+t73;
  t77 = t34+t43+t47+t59+t67+t73;
  t78 = t38+t48+t52+t60+t67+t70+t72+t73;
  A0[0][0] = t17*7.76008788610026E-4+t44+t46*3.799806906686181E-4+t49+t68-cos(t12*2.0+t29)*1.89990345334309E-4-cos(t12*2.0-9.501914794750611E-1)*6.405061947617221E-2-cos(q_4+t12*2.0-1.052751428494702)*1.318047347898018E-3-cos(q_2+q_4+t12-3.430239404207034E-1)*1.164013182915039E-3+cos(q_2+t12+t27+3.430239404207034E-1)*1.164013182915039E-3+cos(t12*2.0+t27+4.257188103684256E-1)*1.003219223949083E-3+cos(q_2-1.564997896182638)*4.490895495778097E-1-cos(t13+6.036381913644014E-3)*3.958448198563897E-1-cos(t12+7.952052255671733E-1)*1.234041994109196E-1-cos(t8+t12)*6.46673990508355E-4-cos(t10+t13)*2.221275589396552E-1-cos(t12*2.0-t15+1.068495677443165E-2)*1.89990345334309E-4-cos(t12+t27-1.227772386374193)*6.46673990508355E-4+6.782344255E-1;
  A0[0][1] = t78;
  A0[0][2] = t76;
  A0[0][3] = t77;
  A0[1][0] = t78;
  A0[1][1] = t19*4.442551178793104E-1+t64-cos(q_3+t27+3.430239404207034E-1)*2.328026365830078E-3+cos(q_4+1.227772386374193)*1.552017577220052E-3+cos(q_3+q_4-3.430239404207034E-1)*2.328026365830078E-3+9.96340487E-1;
  A0[1][2] = t75;
  A0[1][3] = t74;
  A0[2][0] = t76;
  A0[2][1] = t75;
  A0[2][2] = cos(q_4+1.227772386374193)*1.552017577220052E-3-cos(t6-1.068495677443165E-2)*7.599613813372361E-4+1.96250007E-1;
  A0[2][3] = t36;
  A0[3][0] = t77;
  A0[3][1] = t74;
  A0[3][2] = t36;
  A0[3][3] = 5.825636E-3;
