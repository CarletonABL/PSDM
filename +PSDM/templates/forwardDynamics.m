function Qdd = forwardDynamics(Q, Qd, tau)

N = size(Q, 2);
assert(size(Q, 2)>=1);
assert(size(Qd, 2)>=1);
assert(size(tau, 2)>=1);

Qdd = coder.nullcopy(zeros(DOF, N));
tau_ind = coder.nullcopy(zeros(DOF, 1));
D = coder.nullcopy(zeros(DOF, DOF));

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


Qdd(:, i) = D \ (tau(:, i) - tau_ind);

end


