function mask = findYpMask(DH_ext, X, g_in, Em, typeID, tol_in, v_in)
    % FINDYPMASK Finds the correlation mask for an exponent map E
    % and a robot DH_ext, X.
    %
    % mask = findYpMask(DH_ext, X, g_in, Em, typeID, tol_in, v_in)
    %
    %   INPUTS:
    %       -DH_ext, X, g: See runID documentation.
    %       -E: The exponent mask
    %       -tol: The tolerance to use
    %       -typeID: 
    %
    %   OUTPUTS:
    %       - mask: A 1 x M mask of which terms of E are correlated to tau.
    
    %% Process Inputs
    
    % Use zero gravity for acceleration
    if isempty(g_in)
        g = [0;0;0];
    else
        g = g_in;
    end
    
    % Parse tolerance
    if nargin < 6 || isempty(tol_in)
        tol = 1e-11;
    else
        tol = tol_in;
    end
    
    % Parse verbosity
    if nargin < 7 || isempty(v_in)
        v = true;
    else
        v = v_in;
    end
    
    %% Start Function

    time = tic;
    
    % Number of terms
    Nterms = size(Em, 2);

    % Determine number of tests to run. If a small number, use a larger
    % number than required to get a more accurate and better conditioned
    % matrix. If a very large number, don't do this to save time.
    Ntestmult = 1.2;
    if Nterms < 500
        Ntestmult = 3;
    end
    if Nterms > 5000
        Ntestmult = 1;
    end
    Ntests = round(Nterms*Ntestmult);

    % Generate joint states
    [Q, Qd, Qdd, tau] = PSDM.generateSamples(DH_ext, X, g, Ntests, 1, typeID);
    
    % Make information matrix    
    Y = PSDM.generateYp(Q, Qd, Qdd, Em);

    % Solve for each of the DOFs
    theta = doRegression(Y, tau);

    % Solve for masks
    mask = any( abs(theta) > tol , 2)';
    
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
    utilities.vprint(v, "\t\t%d terms reduced to %d, Took %.2f seconds.\n\t\tMax Reprojection Error: %.3g.\n", ...
        int32(Nterms), ...
        int32(sum(mask)), ...
        toc(time), ...
        max(abs(r), [], 'all'));
    
end

function C = doRegression(A, B)
    % Does the regression C = A\B, but for large matrices (>1000) does it
    % in steps to speed it up.
    
    % Get size
    [N, ~] = size(A);
    
    if N < 800
        % Just do regression directly, its fast
        
        C = A\B;
        return;
        
    else
        % Matrix is large. Do regression in steps.
        % First, do regression with A a square matrix, but use a lower
        % tolerance.
        tol = 1e-9;
        C = A(1:N, :) \ B(1:N, :);
        mask = any( abs(C) > tol, 2);
        
        % eliminate uncorrelated terms
        C(~mask, :) = 0;
        
        % Now, do a "well" conditioned least squared fit on the remaining
        % terms.
        N2 = sum(mask);
        M2 = round(N2*1.2);
        C(mask, :) = A(1:M2, mask) \ B(1:M2, :);
        
        return;
    end

end