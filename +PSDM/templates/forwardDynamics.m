function Qdd = forwardDynamics(Q, Qd, tau, Theta)

N = size(Q, 2);
assert(size(Q, 2)>=1);
assert(size(Qd, 2)>=1);
assert(size(tau, 2)>=1);

Qdd = coder.nullcopy(zeros(DOF, N));
D = coder.nullcopy(zeros(DOF, DOF));

%SETUP1_CODE%

for i = 1:N

Qi = Q(:, i);
Qdi = Qd(:, i);
Qddi = Qdd(:, i);
gi = vertcat( Qi, sin(Qi), cos(Qi), Qdi, Qddi);
%SETUP2_CODE%


%TAU_CODE%


Qdd(:, i) = D \ (tau(:, i) - tau_ind);

end


