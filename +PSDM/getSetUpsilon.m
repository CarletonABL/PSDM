function Up = getSetUpsilon(lt, order)
    % GETUPSILON Produces the upsilon set for a manipulator of order k.
    %
    % Inputs:
    %   -lt: DOF x 1 vector of link types
    %   -Order: the order of the set to produce
    %
    % Output:
    %   -Up: The functions in the set Upsilon(k) for the manipulator,
    %        represented as a (3*DOF)xM integer matrix of exponents on the
    %        functions: Q, sin(Q) and cos(Q), respectively.

    lt = logical(lt);
    DOF = size(lt, 1);
    
    % Get zetas (in loop)
    n = zeros(DOF, 1); % Size of each zeta
    Z = cell(DOF, 1);
    for i = 1:DOF
        Z{i} = PSDM.getSetZ(order, lt(i));
    end
    
    % Get size of zetas in loop
    for i = 1:DOF
        n(i) = size(Z{i}, 2);
    end
    
    % Combine zeta functions together into all possible combinations.
    
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