function Up = getSetUpsilon(DH_ext, order)
    % GETUPSILON Produces the upsilon set for a manipulator of order k.
    %
    % Inputs:
    %   -DH_ext: DOF x 6 matrix of DH parameters in order
    %        [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
    %          :      :        :       :         :         :     
    %         a_n  alpha_n    d_n   theta_n    lt_n    q_sign_n];
    %  -Order: the order of the set to produce
    %
    % Output:
    %   -Up: The functions in the set Upsilon(k) for the manipulator,
    %        represented as a (3*DOF)xM integer matrix of exponents on the
    %        functions: Q, sin(Q) and cos(Q), respectively.

    lt = logical(DH_ext(:, 5));
    DOF = size(DH_ext, 1);
    
    % Get zetas
    n = zeros(DOF, 1); % Size of each zeta
    Z = cell(DOF, 1);
    for i = 1:DOF
        Z{i} = PSDM.getSetZ(order, lt(i));
    end
    
    % Get size of zetas in loop
    for i = 1:DOF
        n(i) = size(Z{i}, 2);
    end
    
    % Combine
    
    % Get total number in set
    M = prod(n);
    Up = zeros(3*DOF, M, 'uint8');
    
    for i = 1:DOF
        ind = (1:3)*DOF + i - DOF;
        n_reps = prod(n( (1:DOF) > i ));
        n_its = prod(n( (1:DOF) < i ));
        Up(ind, :) = repmat(repelem(Z{i}, 1, n_its), [1, n_reps]);
    end
    
end