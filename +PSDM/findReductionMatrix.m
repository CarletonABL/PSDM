function P = findReductionMatrix(robot, Ep, idType_in, P1_in, opt)
    % FINDREDUCTIONMATRIX Identifies linear dependencies in the theta vector of
    % an exponent map E and robot defined by the tuple {DH_ext, X, g}.
    % If the map E is already reduced by a matrix P, supply that as P1.
    %
    % P = findReductionMatrix(robot, Ep, idType, P1, opt)

    %% Process inputs
    
    DOF = robot.DOF;
    coder.extrinsic('utilities.blockprod');
    c = PSDM.config;
    
    % Parse idType
    if nargin < 3 || isempty(idType_in)
        idType = 'gravity';
    else
        idType = idType_in;
    end
        
    % Fill in P1, also check size
    if nargin < 4 || isempty(P1_in)
        P1 = repmat(eye(size(Ep, 2)), [1 1 DOF]);
        beCareful = false;
    else
        P1 = P1_in;
        beCareful = true;
    end
    assert(size(P1, 1) == size(Ep, 2), "Mismatched P1 and E sizes!");
    
    % Exit out if empty E matrix is given.
    if size(Ep, 2) == 0
        P = P1;
        return;
    end
    
    %% Start function
    coder.extrinsic('tic', 'toc')
    t = tic;
    
    % Get size of incoming terms.
    M = size(P1, 2);
    
    % Number of joint states to test at. Use a number greater than
    % required to reduce error
    Nq = max(round(M*(1.2 + 1*beCareful)), DOF*30);

    % Max number of inertial parameters is DOF*10
    Nt = round(DOF * 10);
    
    % Generate samples
    [Q, Qd, Qdd, tau] = PSDM.generateSamples(robot, Nq, Nt, idType);
    
    % Make Y matrix, transform it with P1, if given
    Y = PSDM.generateYp(Q, Qd, Qdd, Ep);
        
    % Solve each regression problem in a loop
    % If Yi is the same for all i, solve this in a single step
    if beCareful
        Yi = coder.nullcopy(zeros(size(Y, 1), size(P1, 2), size(P1, 3)));
        Yi = utilities.blockprod(Y, P1);
        Ti = coder.nullcopy(zeros( M, Nt, DOF ));
        for i = 1:DOF
            Ti(:, :, i) = utilities.linsolve( Yi(:, :, i), squeeze(tau(:, i, :)) );
        end
    else
        Ti_horzstack = linsolve( Y, utilities.vertStack(permute(tau, [3, 1, 2]))');
        Ti = reshape(Ti_horzstack, [M, Nt, DOF]);
        if c.do_reprojection_tests
            Yi = Y;
        end
    end
        
    T_star = utilities.vertStack(Ti, 3);
        
    % Reduce to minimum parameters. Since we stacked all Theta vectors
    % vertically, the result will be a vertical stacking of the Pi
    % vectors.
    B_star = utilities.rrefQR(T_star');
    
    % The rank of the matrix
    b = size(B_star, 1);
    
    % Get the pseudoinverse.
    P_star = utilities.pinv(B_star);
    
    % Un-stack the matrix in a loop
    P2 = zeros(M, b, DOF);
    for i = 1:DOF
        P2(:, :, i) = P_star((1:M)+(i-1)*M, :);
    end
    
    % Combine P1, P2 into final P matrix
    if beCareful
        P = utilities.blockprod(P1, P2);
    else
        P = P2;
    end
    
    % Output information, if required.
    utilities.vprint(opt.v, "\t\tReduced from (%d / %d) to %d parameters (took %.3g seconds).\n", ...
            int32(size(P1, 1)), int32(M), int32(b), toc(t));

    if c.do_reprojection_tests
        %% Double check reprojection error to make sure no errors occured

        % Transform with P reduction matrices.
        % Find theta vector for each DOF of matrix
        tau_stack = permute( ...
                        utilities.vertStack(tau, 2), ...
                        [1 3 2]);
             
        Ymin = coder.nullcopy(zeros(size(Yi, 1), size(P2, 2), size(P2, 3)));
        Ymin = utilities.blockprod(Yi, P2);
        Ymin_stack = utilities.vertStack( Ymin );
        Tmin = B_star * T_star;

        % Find reprojection error
        r = Ymin_stack*Tmin - tau_stack;

        if any(abs(r(:)) > 1e-3, 1)
            if coder.target('matlab')
                warning("Reprojection test failed, max reprojection error is %.3g. Continue?", max(abs(r), [], 'all'));
                keyboard;
            else
                fprintf("Reprojection test failed, max reprojection error is %.3g!\n PROCESS FAILED.\n\n", max(abs(r), [], 'all'));
            end
        end
        
        utilities.vprint(opt.v, "\t\tReprojection error is %.3g\n", max(abs(r(:))) );
        
    end
    
    
        
end
