function P = findBaseParams(DH_ext, X, g, E, idType_in, P1_in, tol_in,  v_in)
    % FINDBASEPARAMS Identifies linear dependencies in the theta vector of
    % an exponent map E and robot defined by the tuple {DH_ext, X, g}.
    % If the map E is already reduced by a matrix P, supply that as P1.

    DOF = size(DH_ext, 1);
    
    %% Process inputs
    if nargin < 5 || isempty(idType_in)
        idType = 'gravity';
    else
        idType = idType_in;
    end
    
    % Fill in P1, also check size
    if nargin < 6 || isempty(P1_in)
        P1 = repmat(eye(size(E, 2)), [1 1 DOF]);
    else
        P1 = P1_in;
    end
    assert(size(P1, 1) == size(E, 2), "Mismatched P1 and E sizes!");

    
    if nargin < 7 || isempty(tol_in)
        tol = 1e-14;
    else
        tol = tol_in;
    end
    
    if nargin < 8 || isempty(v_in)
        v = true;
    else
        v = v_in;
    end
    
    %% Start function

    t = tic;
    
    % Define some constants
    DOF = size(DH_ext, 1);
    
    M = size(P1, 2);
    Nq = max(round(M * 2), DOF*10*2);
    Nt = round(DOF * 10); %Magic number is 243 for accel (142/28 terms), and 25 for grav (15/6 terms)
    
    [Q, Qd, Qdd, tau] = PSDM.genTestPoses(DH_ext, X, g, Nq, Nt, idType);
    
    % Make Y matrix
    Y = PSDM.genTermValues(Q, Qd, Qdd, E);
    Yi = utils.blockprod(Y, P1);
    
    % Y tile is Y for each joint
    Ytile = tileArray(Yi, DOF);
    
    tau_stack = permute( ...
                    utils.vertStack(tau, 2), ...
                    [1 3 2]);
   
    % Solve
    if coder.target('matlab'); warnStruct = warning('off', 'MATLAB:rankDeficientMatrix'); end
    T = Ytile \ tau_stack;
    if coder.target('matlab'); warning(warnStruct); end

    % Round out any columns smaller than a tolerance
    T(:, all(abs(T) < tol, 1)) = 0;
    
    % Reduce to minimum parameters    
    P_stack_inv = utils.rref(T', [], true);
    
    b = size(P_stack_inv, 1);
    
    P_stack = pinv(P_stack_inv);
    
    P2 = zeros(M, b, DOF);
    for i = 1:DOF
        P2(:, :, i) = P_stack((1:M)+(i-1)*M, :);
    end

    % Find new Y and T
    Ymin = Ytile * P_stack;
    Tmin = P_stack_inv * T;
    
    P = utils.blockprod(P1, P2);

    % Find reprojection error
    r = Ymin*Tmin - tau_stack;
    
    if any(abs(r) > 1e-3, 'all')
        if coder.target('matlab')
            warning("Reprojection test failed, max reprojection error is %.3g. Continue?", max(abs(r), [], 'all'));
            keyboard;
        else
            fprintf("Reprojection test failed, max reprojection error is %.3g!\n PROCESS FAILED.\n\n", max(abs(r), [], 'all'));
        end
    end
    
    utils.vprint(v, "\t\tReduced from (%d / %d) to %d parameters (took %.3g seconds).\n\t\tReprojection error is %.3g\n", ...
        int32(size(P1, 1)), int32(M), int32(b), toc(t), max(abs(r), [], 'all'));
        
end


function B = tileArray(A, N)
    % Takes an array a and repeats it along the diagonal n times.
    % If A is a page array, then each tile is the appropriate lement of A
    
    [n, m, p] = size(A);
    B = zeros( n*N, m*N );
    
    for i = 1:N
        if p > 1
            B( (1:n)+(i-1)*n, (1:m)+(i-1)*m ) = A(:, :, i);
        else
            B( (1:n)+(i-1)*n, (1:m)+(i-1)*m ) = A(:, :, 1);
        end
    end

end