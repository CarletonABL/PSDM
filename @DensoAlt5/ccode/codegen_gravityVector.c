  /*
  Inputs: Q.
  */
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
  t19 = t2*t5*t7*-9.7708299248E-2;
  t20 = t2*t5*t7*9.7708299248E-2;
  t21 = t3*t5*t6*-9.7708299248E-2;
  t22 = t3*t5*t6*9.7708299248E-2;
  t23 = t2*t7*t9*9.21776408E-4;
  t24 = t3*t6*t9*9.21776408E-4;
  t25 = t2*t3*t4*t9*-9.7708299248E-2;
  t26 = t2*t3*t4*t9*9.7708299248E-2;
  t27 = t2*t3*t4*t5*-9.21776408E-4;
  t28 = t2*t3*t4*t5*9.21776408E-4;
  t29 = t4*t6*t7*t9*9.7708299248E-2;
  t30 = t4*t5*t6*t7*9.21776408E-4;
  t13 = t12*-8.47946040172;
  t14 = t12*8.47946040172;
  t15 = t11*6.4779307992;
  t16 = t8*t11*-3.5615871424E-1;
  t17 = t8*t11*3.5615871424E-1;
  t18 = t4*t11*2.84377828E-2;
  A0[1][0] = t2*-1.7023445152E-1-t6*3.184717877376E+1+t13+t15+t16+t18+t19+t21+t23+t24+t25+t27+t29+t30;
  A0[2][0] = t13+t15+t16+t18+t19+t21+t23+t24+t25+t27+t29+t30;
  A0[3][0] = t4*t12*-3.5615871424E-1-t8*t12*2.84377828E-2+t2*t5*t7*t8*9.21776408E-4+t3*t5*t6*t8*9.21776408E-4+t2*t7*t8*t9*9.7708299248E-2+t3*t6*t8*t9*9.7708299248E-2;
  A0[4][0] = t2*t3*t5*-9.21776408E-4-t2*t3*t9*9.7708299248E-2+t5*t6*t7*9.21776408E-4+t6*t7*t9*9.7708299248E-2-t2*t4*t5*t7*9.7708299248E-2-t3*t4*t5*t6*9.7708299248E-2+t2*t4*t7*t9*9.21776408E-4+t3*t4*t6*t9*9.21776408E-4;
