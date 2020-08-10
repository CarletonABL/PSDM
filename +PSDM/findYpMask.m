function mask = findYpMask(robot, Em, typeID, opt)
    % FINDYPMASK Finds the correlation mask for an exponent map E
    % and a robot DH_ext, X.
    %
    % mask = findYpMask(robot, Em, typeID, o)
    %
    % mask is 1 x M mask of which terms of E are correlated to tau.

    time = tic;
    
    % Number of terms
    m = size(Em, 2);

    % Determine number of tests to run. If a small number, use a larger
    % number than required to get a more accurate and better conditioned
    % matrix. If a very large number, don't do this to save time.
    Ntestmult = 2;
    if m < 500
        Ntestmult = 3;
    end
    if m > 5000
        Ntestmult = 1;
    end
    Ntests = round(m*Ntestmult);

    % Generate joint states
    [Q, Qd, Qdd, tau] = PSDM.generateSamples(robot, Ntests, 1, typeID);
    
    % Make information matrix    
    Y = PSDM.generateYp(Q, Qd, Qdd, Em);

    % Solve for each of the DOFs
    theta = doRegression(Y, tau, opt.tol);

    % Solve for masks
    mask = any( abs(theta) > opt.tol , 2)';
    
    % Output information, if required.
    utilities.vprint(opt.v, "\t\t%d terms reduced to %d, Took %.2f seconds.\n", ...
        int32(m), ...
        int32(sum(mask)), ...
        toc(time));
    
    c = PSDM.config;
    if c.do_reprojection_tests
    
        % Check reprojection
        r = Y(:, mask) * theta(mask, :) - tau;

        if any(abs(r) > 1e-3, 'all')
            if coder.target('matlab')
                warning("Reprojection test failed, max reprojection error is %.3g. Continue?", max(abs(r), [], 'all'));
                keyboard;
            else
                fprintf("Reprojection test failed, max reprojection error is %.3g!\n PROCESS FAILED.\n\n", max(abs(r), [], 'all'));
            end
        end
        
        % Output information, if required.
        utilities.vprint(opt.v, "\t\tMax Reprojection Error: %.3g.\n",  max(abs(r), [], 'all'));
    
    end
    
end

function C = doRegression(A, B, tol)
    % Does the regression C = A\B, but for large matrices (>1000) does it
    % in steps to speed it up.
        
    % Get size
    DOF = size(B, 2);
    [n, m] = size(A);
        
    if n < 800
        % Just do regression directly, its fast
        
        C = utilities.linsolve(A, B);
        return;
        
    else
        % Matrix is large. Do regression in steps.
        % First, do regression with A a square matrix, but use a lower
        % tolerance.
        
        mask = ones(m, 1, 'logical');
        for i = 1:2
            N = sum(mask);
            
            % If mask is empty, just break now
            if ~any(mask)
                break;
            end
            
            C = utilities.linsolve(A(1:N, mask), B(1:N, :));

            % Find tolerance
            Cmax = max(abs(C), [], 2);
            Cmax_sort = sort(Cmax);
            tol_i = min(Cmax_sort( round( N*0.66 ) ) + eps*10, tol/10);
            mask(mask) = Cmax > tol_i;
            
            if tol_i == tol/10
                break;
            end
            
        end
        
        % Now, do a "well" conditioned least squared fit on the remaining
        % terms.
        N = round( min(sum(mask) * 2, n));
        
        C = zeros(m, DOF);
        
        % Do regression (only if mask has elements to regress)
        if any(mask)
            C(mask, :) = utilities.linsolve(A(1:N, mask), B(1:N, :));
        end
        
        return;
        
    end

end