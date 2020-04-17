function P = findReductionMatrix(DH_ext, X, g, Ep, idType_in, P1_in, tol_in,  v_in)
    % FINDREDUCTIONMATRIX Identifies linear dependencies in the theta vector of
    % an exponent map E and robot defined by the tuple {DH_ext, X, g}.
    % If the map E is already reduced by a matrix P, supply that as P1.
    %
    % P = findReductionMatrix(DH_ext, X, g, Ep, idType, P1, tol,  v)

    %% Process inputs
    
    DOF = size(DH_ext, 1);
    
    % Parse idType
    if nargin < 5 || isempty(idType_in)
        idType = 'gravity';
    else
        idType = idType_in;
    end
    
    % Parse tolerance
    if nargin < 7 || isempty(tol_in)
        tol = 1e-12;
    else
        tol = tol_in;
    end
    
    % Fill in P1, also check size
    if nargin < 6 || isempty(P1_in)
        P1 = repmat(eye(size(Ep, 2)), [1 1 DOF]);
        beCareful = false;
    else
        P1 = P1_in;
        m = size(Ep, 2);
        beCareful = ~all(size(P1) == [m, m, DOF]) || ...
            utils.eqtol(P1, repmat(eye(m), [1 1 DOF]), tol, true);
    end
    assert(size(P1, 1) == size(Ep, 2), "Mismatched P1 and E sizes!");

    
    % Parse verbosity
    if nargin < 8 || isempty(v_in)
        v = true;
    else
        v = v_in;
    end
    
    % Exit out if empty E matrix is given.
    if size(Ep, 2) == 0
        P = P1;
        return;
    end
    
    %% Start function
    t = tic;
    
    % Define some constants
    DOF = size(DH_ext, 1);
    
    % Get size of incoming terms.
    M = size(P1, 2);
    
    % Number of joint states to test at. Use a number greater than
    % required to reduce error
    % Nq = max(round(M * 2), DOF*10*2);
    Nq = max(round(M*(1.2 + 2*beCareful)), DOF*10*2);
    
    % Max number of inertial parameters is DOF*10
    Nt = round(DOF * 10); 
    
    % Generate samples
    [Q, Qd, Qdd, tau] = PSDM.generateSamples(DH_ext, X, g, Nq, Nt, idType);
    
    % Make Y matrix, transform it with P1, if given
    Y = PSDM.generateYp(Q, Qd, Qdd, Ep);
    Yi = utilities.blockprod(Y, P1);
    
    % Find theta vector for each DOF of matrix
    Ytile = tileArray(Yi, DOF);
    tau_stack = permute( ...
                    utilities.vertStack(tau, 2), ...
                    [1 3 2]);
    
    % Solve each regression problem in a loop
    Ti = coder.nullcopy(zeros( M, Nt, DOF ));
    for i = 1:DOF
        Ti(:, :, i) = doRegression(Yi(:, :, i), squeeze(tau(:, i, :)), tol);
    end
    
    % Stack T vectors vertically
    T = utilities.vertStack(Ti, 3);
    
    % Round out any columns smaller than a tolerance
    T(:, all(abs(T) < tol, 1)) = 0;
    
    % Reduce to minimum parameters. Since we stacked all Theta vectors
    % vertically, the result will be a vertical stacking of the P_inv
    % vectors.
    P_stack_inv = utilities.rref(T', [], true);
    
    % The rank of the matrix
    b = size(P_stack_inv, 1);
    
    % Get the pseudoinverse.
    P_stack = pinv(P_stack_inv);
    
    % Un-stack the matrix in a loop
    P2 = zeros(M, b, DOF);
    for i = 1:DOF
        P2(:, :, i) = P_stack((1:M)+(i-1)*M, :);
    end
    
    % Combine P1, P2 into final P matrix
    P = utilities.blockprod(P1, P2);

    %% Double check reprojection error to make sure no errors occured
        
    % Transform with P reduction matrices.
    Ymin = Ytile * P_stack;
    Tmin = P_stack_inv * T;
    
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
    
    % Output information, if required.
    utilities.vprint(v, "\t\tReduced from (%d / %d) to %d parameters (took %.3g seconds).\n\t\tReprojection error is %.3g\n", ...
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


function T = doRegression(Y, tau, tol)
    % Performs the regression Y\tau, but removes any "zero" columns from Y
    % first, which prevents ill-defined results.

    if nargin < 3 || isempty(tol)
        tol = 1e-11;
    end
    
    T = zeros(size(Y, 2),  size(tau, 2));
    nonzero_mask = any( abs( Y ) > tol , 1 );
        
    T(nonzero_mask, :) = linsolve( Y(:, nonzero_mask), tau );

end