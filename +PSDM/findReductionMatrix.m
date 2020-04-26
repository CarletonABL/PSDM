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
        beCareful = true;
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
    Nq = max(round(M*(1.2 + 1*beCareful)), DOF*30);

    % Max number of inertial parameters is DOF*10
    Nt = round(DOF * 10);
    
    % Generate samples
    [B_stack, Qd, Qdd, tau] = PSDM.generateSamples(DH_ext, X, g, Nq, Nt, idType);
    
    % Make Y matrix, transform it with P1, if given
    Y = PSDM.generateYp(B_stack, Qd, Qdd, Ep);
    Yi = utilities.blockprod(Y, P1);
        
    % Solve each regression problem in a loop
    % If Yi is the same for all i, solve this in a single step
    if beCareful
        Ti = coder.nullcopy(zeros( M, Nt, DOF ));
        for i = 1:DOF
            Ti(:, :, i) = linsolve( Yi(:, :, i), squeeze(tau(:, i, :)) );
        end
    else
        Ti_horzstack = linsolve( Yi(:, :, 1), utils.vertStack(permute(tau, [3, 1, 2]))');
        Ti = zeros(M, Nt, DOF);
        for i = 1:DOF
            Ti(:, :, i) = Ti_horzstack(:, (i-1)*Nt + (1:Nt));
        end
    end
        
    T = utilities.vertStack(Ti, 3);
    
    
    % Reduce to minimum parameters. Since we stacked all Theta vectors
    % vertically, the result will be a vertical stacking of the P_inv
    % vectors.
    %  B_stack = utilities.rref(T', [], true);
    c = PSDM.config;
    B_stack = utilities.rrefQR(T', [], true, c.use_iterative_refinement);
    
    
    % The rank of the matrix
    b = size(B_stack, 1);
    
    % Get the pseudoinverse.
    P_stack = pinv(B_stack);
    
    % Un-stack the matrix in a loop
    P2 = zeros(M, b, DOF);
    for i = 1:DOF
        P2(:, :, i) = P_stack((1:M)+(i-1)*M, :);
    end
    
    % Combine P1, P2 into final P matrix
    P = utilities.blockprod(P1, P2);
    
    % Output information, if required.
    utilities.vprint(v, "\t\tReduced from (%d / %d) to %d parameters (took %.3g seconds).\n", ...
            int32(size(P1, 1)), int32(M), int32(b), toc(t));

    if c.do_reprojection_tests
        %% Double check reprojection error to make sure no errors occured

        % Transform with P reduction matrices.
        % Find theta vector for each DOF of matrix
        tau_stack = permute( ...
                        utilities.vertStack(tau, 2), ...
                        [1 3 2]);

        Ymin = utilities.vertStack( utilities.blockprod(Yi, P2) );
        Tmin = B_stack * T;

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
        
        utilities.vprint(v, "\t\tReprojection error is %.3g\n", max(abs(r), [], 'all'));
        
    end
    
    
        
end
