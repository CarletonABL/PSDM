function Qdd = forwardDynamics(Q, Qd, tau)

g = horzcat( Q', sin(Q)', cos(Q)');

a = horzcat(Qd', ones(size(Qd))');

N = size(Q, 2);
DOF = size(Q, 1);

Qdd = zeros(DOF, N);

%PTHETA_INDUCED_CODE%
%PTHETA_ACCEL_CODE%

for i = 1:N

gi = g(i, :);
ai = a(i, :);

%SETUP_CODE%


%UP_CODE%


%A_CODE%


%Y_INDUCED_CODE%


%UP_ACCEL_CODE%


%D_CODE%

tau_induced = (Yp_induced_i * PTheta_induced)';

Qdd(:, i) = Di \ (tau(:, i) - tau_induced);

end


