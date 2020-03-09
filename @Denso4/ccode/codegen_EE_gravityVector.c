  /*
  Inputs: Q, Xtool.
  */
  t2 = cos(q_4);
  t3 = sin(q_4);
  t4 = q_2+q_3;
  t5 = cos(t4);
  t6 = sin(t4);
  t7 = t6*-2.95E-1;
  t8 = t6*2.95E-1;
  t9 = t5*9.0E-2;
  t10 = r_c_tool_y*t6;
  t11 = r_c_tool_z*t3*t5;
  t12 = r_c_tool_x*t2*t5*-1.0;
  t13 = r_c_tool_x*t2*t5*1.0;
  A0[1][0] = m_tool*(t7+t9+t10+t11+t12-sin(q_2)*2.7E-1)*9.806132;
  A0[2][0] = m_tool*(t7+t9+t10+t11+t12)*9.806132;
  A0[3][0] = m_tool*t6*(r_c_tool_x*t3+r_c_tool_z*t2)*9.806132;
