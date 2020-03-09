function D = codegen_EE_massMatrix(in1,in2)
%CODEGEN_EE_MASSMATRIX
%    D = CODEGEN_EE_MASSMATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    29-Feb-2020 11:53:35

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
q_5 = in1(5,:);
r_c_tool_x = in2(:,2);
r_c_tool_y = in2(:,3);
r_c_tool_z = in2(:,4);
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
t12 = q_5.*2.0;
t13 = m_tool.*9.5125e-2;
t14 = q_2+q_3;
t15 = r_c_tool_x.^2;
t16 = r_c_tool_y.^2;
t17 = r_c_tool_z.^2;
t18 = t5.^2;
t19 = t6.^2;
t20 = I_yy_tool.*t5;
t21 = I_yz_tool.*t6;
t22 = sin(t12);
t23 = t8.*2.7e-1;
t24 = I_xy_tool.*t11;
t25 = t14+1.274681180919932;
t26 = sin(t14);
t27 = m_tool.*t9.*2.43e-2;
t28 = m_tool.*t15;
t29 = m_tool.*t16;
t30 = m_tool.*t17;
t31 = I_xz_tool.*t10.*-1.0;
t32 = I_xz_tool.*t10.*1.0;
t33 = m_tool.*t4.*7.965e-2;
t36 = I_xx_tool.*t3.*t4;
t38 = m_tool.*r_c_tool_y.*t5.*-2.95e-1;
t39 = m_tool.*r_c_tool_y.*t5.*2.95e-1;
t40 = m_tool.*r_c_tool_z.*t6.*5.9e-1;
t41 = I_xy_tool.*t6.*t10;
t42 = m_tool.*r_c_tool_x.*t11.*-5.9e-1;
t43 = m_tool.*r_c_tool_x.*t11.*5.9e-1;
t44 = m_tool.*r_c_tool_x.*t6.*-9.0e-2;
t45 = m_tool.*r_c_tool_x.*t6.*9.0e-2;
t46 = m_tool.*r_c_tool_y.*t10.*-7.5e-2;
t47 = m_tool.*r_c_tool_y.*t10.*7.5e-2;
t48 = I_yz_tool.*t10.*t11;
t49 = m_tool.*r_c_tool_y.*t10.*1.8e-1;
t50 = m_tool.*r_c_tool_z.*t11.*-9.0e-2;
t51 = m_tool.*r_c_tool_z.*t11.*9.0e-2;
t52 = m_tool.*r_c_tool_y.*r_c_tool_z.*t6;
t54 = m_tool.*r_c_tool_x.*r_c_tool_y.*t11;
t55 = m_tool.*r_c_tool_x.*r_c_tool_z.*t10;
t68 = I_xx_tool.*t8.*t9.*-1.0;
t69 = I_xx_tool.*t8.*t9.*1.0;
t71 = m_tool.*r_c_tool_y.*t3.*t5.*-2.7e-1;
t72 = m_tool.*r_c_tool_y.*t3.*t5.*2.7e-1;
t73 = m_tool.*r_c_tool_y.*t4.*t5.*-2.7e-1;
t74 = m_tool.*r_c_tool_y.*t4.*t5.*2.7e-1;
t75 = m_tool.*r_c_tool_z.*t4.*t6.*2.7e-1;
t76 = m_tool.*r_c_tool_z.*t5.*t6.*2.95e-1;
t77 = m_tool.*r_c_tool_x.*t5.*t6.*7.5e-2;
t78 = I_xy_tool.*t3.*t6.*t9;
t79 = I_xy_tool.*t4.*t6.*t8;
t80 = I_xz_tool.*t3.*t5.*t9;
t81 = I_xz_tool.*t4.*t5.*t8;
t82 = m_tool.*r_c_tool_x.*t4.*t11.*-2.7e-1;
t83 = m_tool.*r_c_tool_x.*t4.*t11.*2.7e-1;
t84 = m_tool.*r_c_tool_x.*t6.*t9.*-2.7e-1;
t85 = m_tool.*r_c_tool_x.*t6.*t9.*2.7e-1;
t86 = m_tool.*r_c_tool_x.*t5.*t11.*-2.95e-1;
t87 = m_tool.*r_c_tool_x.*t5.*t11.*2.95e-1;
t88 = m_tool.*r_c_tool_x.*t6.*t10.*-2.95e-1;
t89 = m_tool.*r_c_tool_x.*t6.*t10.*2.95e-1;
t90 = m_tool.*r_c_tool_x.*t5.*t6.*-1.8e-1;
t91 = m_tool.*r_c_tool_x.*t5.*t6.*1.8e-1;
t92 = m_tool.*r_c_tool_z.*t5.*t11.*7.5e-2;
t93 = m_tool.*r_c_tool_z.*t6.*t10.*7.5e-2;
t94 = I_yy_tool.*t3.*t9.*t10;
t95 = I_yy_tool.*t4.*t8.*t10;
t96 = I_yz_tool.*t3.*t9.*t11;
t97 = I_yz_tool.*t4.*t8.*t11;
t98 = I_zz_tool.*t6.*t10.*t11;
t99 = m_tool.*r_c_tool_y.*t8.*t10.*-2.7e-1;
t101 = m_tool.*r_c_tool_y.*t9.*t10.*2.7e-1;
t102 = m_tool.*r_c_tool_z.*t9.*t11.*-2.7e-1;
t103 = m_tool.*r_c_tool_z.*t9.*t11.*2.7e-1;
t104 = m_tool.*r_c_tool_z.*t10.*t11.*-2.95e-1;
t105 = m_tool.*r_c_tool_z.*t10.*t11.*2.95e-1;
t106 = m_tool.*r_c_tool_z.*t5.*t11.*-1.8e-1;
t107 = m_tool.*r_c_tool_z.*t5.*t11.*1.8e-1;
t108 = m_tool.*r_c_tool_x.*t10.*t11.*-7.5e-2;
t109 = m_tool.*r_c_tool_x.*t10.*t11.*7.5e-2;
t111 = I_xz_tool.*t8.*t9.*t10;
t124 = I_xx_tool.*t6.*t10.*t11.*-1.0;
t125 = I_xx_tool.*t6.*t10.*t11.*1.0;
t131 = m_tool.*r_c_tool_x.*r_c_tool_y.*t6.*t10.*-1.0;
t132 = m_tool.*r_c_tool_x.*r_c_tool_y.*t6.*t10.*1.0;
t133 = m_tool.*r_c_tool_y.*r_c_tool_z.*t10.*t11.*-1.0;
t134 = m_tool.*r_c_tool_y.*r_c_tool_z.*t10.*t11.*1.0;
t141 = m_tool.*r_c_tool_x.*t3.*t6.*t10.*-2.7e-1;
t142 = m_tool.*r_c_tool_x.*t3.*t6.*t10.*2.7e-1;
t146 = m_tool.*r_c_tool_x.*t4.*t6.*t10.*-2.7e-1;
t147 = m_tool.*r_c_tool_x.*t4.*t6.*t10.*2.7e-1;
t158 = m_tool.*r_c_tool_z.*t3.*t10.*t11.*-2.7e-1;
t159 = m_tool.*r_c_tool_z.*t3.*t10.*t11.*2.7e-1;
t162 = m_tool.*r_c_tool_z.*t4.*t10.*t11.*-2.7e-1;
t163 = m_tool.*r_c_tool_z.*t4.*t10.*t11.*2.7e-1;
t166 = m_tool.*r_c_tool_y.*t3.*t9.*t10.*-2.95e-1;
t167 = m_tool.*r_c_tool_y.*t3.*t9.*t10.*2.95e-1;
t168 = m_tool.*r_c_tool_y.*t4.*t8.*t10.*-2.95e-1;
t169 = m_tool.*r_c_tool_y.*t4.*t8.*t10.*2.95e-1;
t171 = m_tool.*r_c_tool_y.*t3.*t4.*t10.*9.0e-2;
t172 = m_tool.*r_c_tool_y.*t3.*t5.*t9.*-9.0e-2;
t173 = m_tool.*r_c_tool_y.*t3.*t5.*t9.*9.0e-2;
t174 = m_tool.*r_c_tool_y.*t4.*t5.*t8.*-9.0e-2;
t175 = m_tool.*r_c_tool_y.*t4.*t5.*t8.*9.0e-2;
t177 = m_tool.*r_c_tool_x.*t8.*t10.*t11.*-2.7e-1;
t181 = m_tool.*r_c_tool_y.*t8.*t9.*t10.*-9.0e-2;
t182 = m_tool.*r_c_tool_y.*t8.*t9.*t10.*9.0e-2;
t204 = I_xz_tool.*t3.*t4.*t6.*t11.*-2.0;
t205 = I_xz_tool.*t3.*t4.*t6.*t11.*2.0;
t214 = I_zz_tool.*t3.*t5.*t9.*t10.*-1.0;
t215 = I_zz_tool.*t3.*t5.*t9.*t10.*1.0;
t216 = I_zz_tool.*t4.*t5.*t8.*t10.*-1.0;
t217 = I_zz_tool.*t4.*t5.*t8.*t10.*1.0;
t222 = I_xz_tool.*t6.*t8.*t9.*t11.*2.0;
t225 = m_tool.*r_c_tool_x.*r_c_tool_y.*t3.*t6.*t9.*-1.0;
t226 = m_tool.*r_c_tool_x.*r_c_tool_y.*t3.*t6.*t9.*1.0;
t227 = m_tool.*r_c_tool_x.*r_c_tool_y.*t4.*t6.*t8.*-1.0;
t228 = m_tool.*r_c_tool_x.*r_c_tool_y.*t4.*t6.*t8.*1.0;
t229 = m_tool.*r_c_tool_x.*r_c_tool_z.*t3.*t5.*t9.*-1.0;
t230 = m_tool.*r_c_tool_x.*r_c_tool_z.*t3.*t5.*t9.*1.0;
t231 = m_tool.*r_c_tool_x.*r_c_tool_z.*t4.*t5.*t8.*-1.0;
t232 = m_tool.*r_c_tool_x.*r_c_tool_z.*t4.*t5.*t8.*1.0;
t233 = m_tool.*r_c_tool_x.*r_c_tool_y.*t5.*t6.*t10.*-2.0;
t234 = m_tool.*r_c_tool_x.*r_c_tool_y.*t5.*t6.*t10.*2.0;
t235 = m_tool.*r_c_tool_y.*r_c_tool_z.*t3.*t9.*t11.*-1.0;
t236 = m_tool.*r_c_tool_y.*r_c_tool_z.*t3.*t9.*t11.*1.0;
t237 = m_tool.*r_c_tool_y.*r_c_tool_z.*t4.*t8.*t11.*-1.0;
t238 = m_tool.*r_c_tool_y.*r_c_tool_z.*t4.*t8.*t11.*1.0;
t239 = m_tool.*r_c_tool_y.*r_c_tool_z.*t5.*t10.*t11.*-2.0;
t240 = m_tool.*r_c_tool_y.*r_c_tool_z.*t5.*t10.*t11.*2.0;
t251 = m_tool.*r_c_tool_x.*t3.*t9.*t10.*t11.*-2.95e-1;
t252 = m_tool.*r_c_tool_x.*t3.*t9.*t10.*t11.*2.95e-1;
t253 = m_tool.*r_c_tool_x.*t4.*t8.*t10.*t11.*-2.95e-1;
t254 = m_tool.*r_c_tool_x.*t4.*t8.*t10.*t11.*2.95e-1;
t256 = m_tool.*r_c_tool_x.*t3.*t4.*t10.*t11.*9.0e-2;
t262 = I_xx_tool.*t6.*t8.*t9.*t10.*t11;
t264 = m_tool.*r_c_tool_x.*r_c_tool_y.*t3.*t5.*t6.*t9;
t265 = m_tool.*r_c_tool_x.*r_c_tool_y.*t4.*t5.*t6.*t8;
t272 = m_tool.*r_c_tool_z.*t6.*t8.*t9.*t10.*9.0e-2;
t273 = m_tool.*r_c_tool_y.*r_c_tool_z.*t3.*t5.*t9.*t11;
t274 = m_tool.*r_c_tool_y.*r_c_tool_z.*t4.*t5.*t8.*t11;
t276 = m_tool.*r_c_tool_x.*t8.*t9.*t10.*t11.*-9.0e-2;
t277 = m_tool.*r_c_tool_x.*t8.*t9.*t10.*t11.*9.0e-2;
t284 = m_tool.*r_c_tool_x.*t3.*t5.*t6.*t9.*2.95e-1;
t285 = m_tool.*r_c_tool_x.*t4.*t5.*t6.*t8.*2.95e-1;
t288 = I_xx_tool.*t3.*t5.*t6.*t9.*t11;
t289 = I_xx_tool.*t4.*t5.*t6.*t8.*t11;
t293 = m_tool.*r_c_tool_z.*t3.*t5.*t9.*t11.*2.95e-1;
t294 = m_tool.*r_c_tool_z.*t3.*t6.*t9.*t10.*2.95e-1;
t295 = m_tool.*r_c_tool_z.*t4.*t5.*t8.*t11.*2.95e-1;
t296 = m_tool.*r_c_tool_z.*t4.*t6.*t8.*t10.*2.95e-1;
t299 = m_tool.*r_c_tool_z.*t3.*t4.*t6.*t10.*-9.0e-2;
t300 = m_tool.*r_c_tool_z.*t3.*t4.*t6.*t10.*9.0e-2;
t306 = I_zz_tool.*t3.*t5.*t6.*t9.*t11.*-1.0;
t307 = I_zz_tool.*t3.*t5.*t6.*t9.*t11.*1.0;
t308 = I_zz_tool.*t4.*t5.*t6.*t8.*t11.*-1.0;
t309 = I_zz_tool.*t4.*t5.*t6.*t8.*t11.*1.0;
t314 = m_tool.*r_c_tool_x.*r_c_tool_z.*t3.*t4.*t6.*t11.*2.0;
t321 = m_tool.*r_c_tool_x.*r_c_tool_z.*t6.*t8.*t9.*t11.*-2.0;
t322 = m_tool.*r_c_tool_x.*r_c_tool_z.*t6.*t8.*t9.*t11.*2.0;
t34 = cos(t25);
t35 = I_xz_tool.*t22;
t37 = t5.*t21;
t53 = -t24;
t56 = I_xx_tool.*t19;
t57 = I_yy_tool.*t18;
t58 = I_zz_tool.*t18.*-1.0;
t59 = I_zz_tool.*t18.*1.0;
t60 = I_zz_tool.*t19.*-1.0;
t61 = I_zz_tool.*t19.*1.0;
t62 = t5.*t28;
t63 = t5.*t30;
t64 = t5.*t24.*-1.0;
t65 = t5.*t24.*1.0;
t70 = t3.*t4.*t21;
t100 = m_tool.*r_c_tool_y.*t10.*t23;
t110 = t8.*t9.*t24;
t112 = t5.*t54;
t113 = -t52;
t114 = I_xz_tool.*t10.*t19.*2.0;
t115 = t3.*t4.*t29;
t116 = t3.*t4.*t30;
t117 = m_tool.*r_c_tool_x.*r_c_tool_z.*t22.*-1.0;
t118 = m_tool.*r_c_tool_x.*r_c_tool_z.*t22.*1.0;
t119 = t3.*t4.*t24.*-1.0;
t120 = t3.*t4.*t24.*1.0;
t121 = t3.*t4.*t31;
t122 = t3.*t4.*t32;
t123 = t5.*t41.*2.0;
t126 = t8.*t9.*t21.*-1.0;
t127 = t8.*t9.*t21.*1.0;
t128 = t5.*t48.*2.0;
t129 = t5.*t52.*-1.0;
t130 = t5.*t52.*1.0;
t135 = t18.*t30;
t136 = t19.*t30;
t137 = t5.*t75;
t138 = t3.*t4.*t38;
t139 = t3.*t4.*t39;
t140 = I_zz_tool.*t18.*t19;
t143 = m_tool.*r_c_tool_x.*t5.*t6.*t23;
t144 = t5.*t82;
t145 = t5.*t83;
t148 = t5.*t84;
t149 = t5.*t85;
t150 = t19.*t28.*-1.0;
t151 = t19.*t28.*1.0;
t152 = t18.*t29.*-1.0;
t153 = t18.*t29.*1.0;
t154 = t3.*t9.*t10.*t20;
t155 = t4.*t8.*t10.*t20;
t156 = t3.*t9.*t10.*t21;
t157 = t4.*t8.*t10.*t21;
t160 = m_tool.*r_c_tool_z.*t5.*t11.*t23;
t161 = m_tool.*r_c_tool_z.*t6.*t10.*t23;
t164 = t5.*t102;
t165 = t5.*t103;
t170 = t8.*t9.*t39;
t178 = m_tool.*r_c_tool_x.*t10.*t11.*t23;
t179 = t3.*t4.*t54;
t180 = t3.*t4.*t55;
t183 = t8.*t9.*t52;
t184 = I_zz_tool.*t3.*t4.*t19;
t186 = t8.*t9.*t29.*-1.0;
t187 = t8.*t9.*t29.*1.0;
t188 = t8.*t9.*t30.*-1.0;
t189 = t8.*t9.*t30.*1.0;
t191 = t19.*t55.*-2.0;
t192 = t19.*t55.*2.0;
t193 = t3.*t9.*t10.*t28;
t194 = t4.*t8.*t10.*t28;
t195 = t3.*t9.*t10.*t30;
t196 = t4.*t8.*t10.*t30;
t197 = t6.*t10.*t11.*t28;
t200 = t5.*t78.*-1.0;
t201 = t5.*t78.*1.0;
t202 = t5.*t79.*-1.0;
t203 = t5.*t79.*1.0;
t208 = t5.*t96.*-1.0;
t209 = t5.*t96.*1.0;
t210 = t5.*t97.*-1.0;
t211 = t5.*t97.*1.0;
t218 = t3.*t9.*t10.*t24.*-1.0;
t219 = t3.*t9.*t10.*t24.*1.0;
t220 = t4.*t8.*t10.*t24.*-1.0;
t221 = t4.*t8.*t10.*t24.*1.0;
t223 = t3.*t4.*t52.*-1.0;
t224 = t3.*t4.*t52.*1.0;
t241 = t8.*t9.*t54.*-1.0;
t242 = t8.*t9.*t54.*1.0;
t243 = t8.*t9.*t55.*-1.0;
t244 = t8.*t9.*t55.*1.0;
t245 = t19.*t36.*-1.0;
t246 = t19.*t36.*1.0;
t247 = I_xz_tool.*t6.*t11.*t18.*-2.0;
t248 = I_xz_tool.*t6.*t11.*t18.*2.0;
t255 = t8.*t9.*t89;
t257 = t3.*t9.*t10.*t44;
t258 = t3.*t9.*t10.*t45;
t259 = t4.*t8.*t10.*t44;
t260 = t4.*t8.*t10.*t45;
t261 = t5.*t8.*t9.*t45;
t266 = t8.*t9.*t105;
t267 = t3.*t9.*t10.*t50;
t268 = t3.*t9.*t10.*t51;
t269 = t4.*t8.*t10.*t50;
t270 = t4.*t8.*t10.*t51;
t271 = t5.*t8.*t9.*t51;
t278 = t3.*t9.*t10.*t54;
t279 = t4.*t8.*t10.*t54;
t280 = t6.*t10.*t11.*t30.*-1.0;
t281 = t6.*t10.*t11.*t30.*1.0;
t282 = t3.*t4.*t88;
t283 = t3.*t4.*t89;
t286 = t3.*t4.*t5.*t44;
t287 = t3.*t4.*t5.*t45;
t290 = t3.*t4.*t98;
t291 = t3.*t4.*t104;
t292 = t3.*t4.*t105;
t297 = t3.*t4.*t5.*t50;
t298 = t3.*t4.*t5.*t51;
t301 = t18.*t19.*t28;
t304 = t6.*t10.*t11.*t36.*-1.0;
t305 = t6.*t10.*t11.*t36.*1.0;
t312 = t8.*t9.*t98.*-1.0;
t313 = t8.*t9.*t98.*1.0;
t315 = t3.*t9.*t10.*t52.*-1.0;
t316 = t3.*t9.*t10.*t52.*1.0;
t317 = t4.*t8.*t10.*t52.*-1.0;
t318 = t4.*t8.*t10.*t52.*1.0;
t323 = t3.*t4.*t19.*t28;
t325 = t18.*t78.*-2.0;
t326 = t18.*t78.*2.0;
t327 = t18.*t79.*-2.0;
t328 = t18.*t79.*2.0;
t330 = t19.*t80.*-2.0;
t331 = t19.*t80.*2.0;
t332 = t19.*t81.*-2.0;
t333 = t19.*t81.*2.0;
t334 = t18.*t96.*-2.0;
t335 = t18.*t96.*2.0;
t336 = t18.*t97.*-2.0;
t337 = t18.*t97.*2.0;
t338 = t19.*t111.*-2.0;
t339 = t19.*t111.*2.0;
t341 = m_tool.*r_c_tool_x.*r_c_tool_z.*t6.*t11.*t18.*2.0;
t344 = I_zz_tool.*t3.*t5.*t9.*t10.*t19;
t345 = I_zz_tool.*t4.*t5.*t8.*t10.*t19;
t346 = t3.*t5.*t9.*t10.*t29.*-1.0;
t347 = t3.*t5.*t9.*t10.*t29.*1.0;
t348 = t4.*t5.*t8.*t10.*t29.*-1.0;
t349 = t4.*t5.*t8.*t10.*t29.*1.0;
t354 = t6.*t8.*t9.*t10.*t11.*t30;
t355 = t6.*t10.*t11.*t80.*-2.0;
t356 = t6.*t10.*t11.*t80.*2.0;
t357 = t6.*t10.*t11.*t81.*-2.0;
t358 = t6.*t10.*t11.*t81.*2.0;
t363 = m_tool.*r_c_tool_x.*r_c_tool_y.*t3.*t6.*t9.*t18.*2.0;
t364 = m_tool.*r_c_tool_x.*r_c_tool_y.*t4.*t6.*t8.*t18.*2.0;
t367 = m_tool.*r_c_tool_x.*r_c_tool_z.*t3.*t5.*t9.*t19.*2.0;
t368 = m_tool.*r_c_tool_x.*r_c_tool_z.*t4.*t5.*t8.*t19.*2.0;
t369 = m_tool.*r_c_tool_y.*r_c_tool_z.*t3.*t9.*t11.*t18.*2.0;
t370 = m_tool.*r_c_tool_y.*r_c_tool_z.*t4.*t8.*t11.*t18.*2.0;
t383 = t3.*t5.*t6.*t9.*t11.*t55.*2.0;
t384 = t4.*t5.*t6.*t8.*t11.*t55.*2.0;
t66 = t34.*-3.084234102658227e-1;
t67 = t34.*3.084234102658227e-1;
t176 = t5.*t110;
t185 = t8.*t9.*t56;
t190 = t3.*t4.*t37;
t198 = t3.*t4.*t64;
t199 = t3.*t4.*t65;
t206 = t18.*t56.*-1.0;
t207 = t18.*t56.*1.0;
t212 = t8.*t9.*t37.*-1.0;
t213 = t8.*t9.*t37.*1.0;
t249 = t8.*t9.*t60;
t250 = t8.*t9.*t61;
t263 = t3.*t4.*t112;
t275 = t5.*t183;
t302 = t3.*t9.*t10.*t63;
t303 = t4.*t8.*t10.*t63;
t310 = t3.*t4.*t129;
t311 = t3.*t4.*t130;
t319 = t8.*t9.*t112.*-1.0;
t320 = t8.*t9.*t112.*1.0;
t324 = t8.*t9.*t136;
t329 = t3.*t4.*t114;
t342 = t19.*t135.*-1.0;
t343 = t19.*t135.*1.0;
t350 = t19.*t116.*-1.0;
t351 = t19.*t116.*1.0;
t352 = t8.*t9.*t150;
t353 = t8.*t9.*t151;
t359 = t3.*t5.*t9.*t10.*t56.*-1.0;
t360 = t3.*t5.*t9.*t10.*t56.*1.0;
t361 = t4.*t5.*t8.*t10.*t56.*-1.0;
t362 = t4.*t5.*t8.*t10.*t56.*1.0;
t365 = t19.*t180.*-2.0;
t366 = t19.*t180.*2.0;
t371 = t8.*t9.*t192;
t372 = t3.*t4.*t197;
t373 = t3.*t6.*t9.*t11.*t63;
t374 = t4.*t6.*t8.*t11.*t63;
t375 = t3.*t6.*t9.*t11.*t62.*-1.0;
t376 = t3.*t6.*t9.*t11.*t62.*1.0;
t377 = t4.*t6.*t8.*t11.*t62.*-1.0;
t378 = t4.*t6.*t8.*t11.*t62.*1.0;
t379 = t6.*t10.*t11.*t116.*-1.0;
t380 = t6.*t10.*t11.*t116.*1.0;
t381 = t8.*t9.*t197.*-1.0;
t382 = t8.*t9.*t197.*1.0;
t385 = t3.*t9.*t10.*t19.*t62;
t386 = t4.*t8.*t10.*t19.*t62;
t391 = t21+t53+t54+t113;
t392 = t20+t41+t44+t48+t50+t62+t63+t76+t86+t131+t133;
t394 = t31+t37+t38+t55+t64+t88+t98+t104+t112+t114+t124+t129+t191+t197+t280;
t397 = t70+t93+t94+t95+t108+t110+t119+t126+t161+t177+t179+t183+t193+t194+t195+t196+t200+t202+t208+t210+t223+t241+t251+t253+t256+t264+t265+t272+t273+t274+t276+t294+t296+t299;
t340 = t23+t66+7.5e-2;
t387 = t19.*t302.*-1.0;
t388 = t19.*t302.*1.0;
t389 = t19.*t303.*-1.0;
t390 = t19.*t303.*1.0;
t393 = t84+t102+t137+t144+t392;
t395 = t73+t146+t162+t394;
t396 = I_zz_tool+t13+t27+t28+t29+t33+t35+t40+t42+t49+t56+t57+t58+t60+t75+t82+t90+t101+t106+t117+t123+t128+t135+t136+t140+t148+t150+t152+t164+t206+t233+t239+t247+t301+t341+t342;
t398 = t36+t46+t68+t77+t80+t81+t92+t99+t115+t116+t143+t156+t157+t160+t166+t168+t171+t181+t184+t185+t186+t188+t204+t218+t220+t222+t229+t231+t245+t249+t261+t271+t278+t279+t284+t285+t286+t288+t289+t293+t295+t297+t306+t308+t314+t315+t317+t321+t323+t324+t330+t332+t350+t352+t367+t368+t373+t374+t375+t377;
t399 = t78+t79+t96+t97+t111+t121+t138+t154+t155+t170+t172+t174+t176+t180+t190+t198+t212+t214+t216+t225+t227+t235+t237+t243+t255+t257+t259+t262+t263+t266+t267+t269+t275+t282+t290+t291+t302+t303+t304+t310+t312+t319+t325+t327+t329+t334+t336+t338+t344+t345+t346+t348+t354+t355+t357+t359+t361+t363+t364+t365+t369+t370+t371+t372+t379+t381+t383+t384+t385+t386+t387+t389;
t400 = t71+t141+t158+t399;
D = reshape([(t3.*t4.*t11-t8.*t9.*t11.*1.0+t3.*t5.*t6.*t9+t4.*t5.*t6.*t8).*(t11.*t36+t11.*t68+t11.*t80+t11.*t81-I_xy_tool.*t10.*t26.*1.0-I_xz_tool.*t3.*t4.*t6.*1.0+I_xz_tool.*t6.*t8.*t9+I_xx_tool.*t3.*t5.*t6.*t9+I_xx_tool.*t4.*t5.*t6.*t8)+(-t3.*t4.*t6+t6.*t8.*t9+t3.*t5.*t9.*t11+t4.*t5.*t8.*t11).*(t6.*t80+t6.*t81-I_yz_tool.*t10.*t26.*1.0+I_xz_tool.*t3.*t4.*t11-I_xz_tool.*t8.*t9.*t11.*1.0-I_zz_tool.*t3.*t4.*t6.*1.0+I_zz_tool.*t6.*t8.*t9+I_zz_tool.*t3.*t5.*t9.*t11+I_zz_tool.*t4.*t5.*t8.*t11)+m_tool.*((r_c_tool_z.*(t7.*t10.*t11.*-1.0+t2.*t3.*t6.*t9+t2.*t4.*t6.*t8+t2.*t3.*t4.*t5.*t11-t2.*t5.*t8.*t9.*t11.*1.0).*1.0-r_c_tool_y.*(t5.*t7+t2.*t3.*t4.*t10-t2.*t8.*t9.*t10.*1.0)+t2.*t340-r_c_tool_x.*(t6.*t7.*t10+t2.*t3.*t9.*t11+t2.*t4.*t8.*t11-t2.*t3.*t4.*t5.*t6+t2.*t5.*t6.*t8.*t9)).^2+(r_c_tool_y.*(t2.*t5-t3.*t4.*t7.*t10.*1.0+t7.*t8.*t9.*t10)+t7.*t340-r_c_tool_x.*(-t2.*t6.*t10+t3.*t7.*t9.*t11+t4.*t7.*t8.*t11-t3.*t4.*t5.*t6.*t7+t5.*t6.*t7.*t8.*t9).*1.0+r_c_tool_z.*(t2.*t10.*t11+t3.*t6.*t7.*t9+t4.*t6.*t7.*t8+t3.*t4.*t5.*t7.*t11-t5.*t7.*t8.*t9.*t11.*1.0)).^2)-t10.*t26.*(t70.*-1.0-t110.*1.0+t5.*t78+t5.*t79+t5.*t96+t5.*t97-I_yy_tool.*t10.*t26.*1.0+t3.*t4.*t24+t8.*t9.*t21).*1.0,t400,t399,t398,t397,t400,I_zz_tool+m_tool.*1.68025e-1+t28+t29+t35+t40+t42+t49+t56+t57+t58+t60+t90+t106+t117+t123+t128+t135+t136+t140+t150+t152+t206+t233+t239+t247+t301+t341+t342+m_tool.*t4.*1.593e-1+m_tool.*t9.*4.86e-2+m_tool.*r_c_tool_z.*t4.*t6.*5.4e-1-m_tool.*r_c_tool_x.*t4.*t11.*5.4e-1+m_tool.*r_c_tool_y.*t9.*t10.*5.4e-1-m_tool.*r_c_tool_x.*t5.*t6.*t9.*5.4e-1-m_tool.*r_c_tool_z.*t5.*t9.*t11.*5.4e-1,t396,t395,t393,t399,t396,I_zz_tool+t13+t28+t29+t35+t40+t42+t49+t56+t57+t58+t60+t90+t106+t117+t123+t128+t135+t136+t140+t150+t152+t206+t233+t239+t247+t301+t341+t342,t394,t392,t398,t395,t394,I_xx_tool+t29+t30-t35.*1.0-t56.*1.0-t136.*1.0+I_zz_tool.*t19+t19.*t28+m_tool.*r_c_tool_x.*r_c_tool_z.*t22,t391,t397,t393,t392,t391,I_yy_tool+t28+t30],[5,5]);
