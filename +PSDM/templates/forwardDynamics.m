function Qdd = forwardDynamics(Q, Qd, tau)

g = vertcat( Q, sin(Q), cos(Q), Qd);

N = size(Q, 2);

Qdd = zeros(DOF, N);
tau_ind = coder.nullcopy(zeros(DOF, 1));
D = coder.nullcopy(zeros(DOF, DOF));

%SETUP1_CODE%

%PHI_CODE%

for i = 1:N

gi = g(:, i);
%SETUP2_CODE%


%UP_CODE%


%A_CODE%


%Y_CODE%


%TAU_CODE%


Qdd(:, i) = D \ (tau(:, i) - tau_ind);

end


