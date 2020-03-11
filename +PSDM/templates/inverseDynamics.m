function tau = inverseDynamics(Q, Qd, Qdd)

g = horzcat( Q', sin(Q)', cos(Q)');

a = horzcat(Qd', Qdd');

N = size(Q, 2);
DOF = size(Q, 1);

tau = zeros(DOF, N);

%PTHETACODE%

for i = 1:N

gi = g(i, :);
ai = a(i, :);

%SETUPCODE%


%UPCODE%


%ACODE%


%YCODE%

tau(:, i) = (Ypi * PTheta)';

end


