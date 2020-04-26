function tau = inverseDynamics(Q, Qd, Qdd)

g = vertcat( Q, sin(Q), cos(Q), Qd, Qdd);

N = size(Q, 2);
DOF = size(Q, 1);

tau = zeros(DOF, N);

%SETUP_CODE%

%PHI_CODE%

for i = 1:N
    
gi1 = g(colMask, i);
gi2 = gi1(squareMask).^2;
   
%UP_CODE%

%A_CODE%

%Y_CODE%

%TAU_CODE%

end

end