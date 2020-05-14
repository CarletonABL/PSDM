function tau = inverseDynamics(Q, Qd, Qdd)

N = size(Q, 2);
DOF = size(Q, 1);

tau = coder.nullcopy(zeros(DOF, N));

%SETUP1_CODE%

%PHI_CODE%

for i = 1:N

Qi = Q(:, i);
Qdi = Qd(:, i);
Qddi = Qdd(:, i);
gi = vertcat( Qi, sin(Qi), cos(Qi), Qdi, Qddi);
%SETUP2_CODE%
   
%UP_CODE%

%A_CODE%

%Y_CODE%

%TAU_CODE%

end

end

%EXTRA_CODE%

