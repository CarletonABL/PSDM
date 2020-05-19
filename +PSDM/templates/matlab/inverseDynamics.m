function tau = inverseDynamics(Q, Qd, Qdd, Theta)

N = size(Q, 2);

%SETUP1_CODE%

for i = 1:N

Qi = Q(:, i);
Qdi = Qd(:, i);
Qddi = Qdd(:, i);
gi = vertcat( Qi, sin(Qi), cos(Qi), Qdi, Qddi);
%SETUP2_CODE%
%TAU_CODE%

end

end

%EXTRA_CODE%

