function mask = findCorrelationMask(DH_ext, X, g_in, Em, typeID, tol_in, v_in)
    % FINDCORRELATIONMASK Finds the correlation mask for an exponent map E
    % and a robot DH_ext, X.
    %
    %   INPUTS:
    %       -DH_ext, X, g: See runID documentation.
    %       -E: The exponent mask
    %       -tol: The tolerance to use
    %       -typeID: 
    %
    %   OUTPUTS:
    %       - mask: A 1 x M mask of which terms of E are correlated to tau.
    %
    
    %% Process Inputs
    
    % Use zero gravity for acceleration
    if isempty(g_in)
        g = [0;0;0];
    else
        g = g_in;
    end
    
    % Fill in arguments
    if nargin < 6 || isempty(tol_in)
        tol = 1e-12;
    else
        tol = tol_in;
    end
    if nargin < 7 || isempty(v_in)
        v = true;
    else
        v = v_in;
    end
    
    %% Start Function
    
    tfunc = tic;
    
    Nterms = size(Em, 2);

    % Number of values to generate
    Ntestmult = 1.2;
    if Nterms < 500
        Ntestmult = 3;
    end
    if Nterms > 5000
        Ntestmult = 1;
    end
    Ntests = round(Nterms*Ntestmult);

    % Generate joint angles
    [Q, Qd, Qdd, tau] = PSDM.genTestPoses(DH_ext, X, g, Ntests, 1, typeID);
    
    % Make information matrix    
    Y = PSDM.genTermValues(Q, Qd, Qdd, Em);

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
    
    utilities.vprint(v, "\t\t%d terms reduced to %d, Took %.2f seconds.\n\t\tMax Reprojection Error: %.3g.\n", ...
        int32(Nterms), ...
        int32(sum(mask)), ...
        toc(tfunc), ...
        max(abs(r), [], 'all'));
    
end

function C = doRegression(A, B)
    % Does the regression C = A\B, but for large matrices (>1000) does it
    % in steps to speed it up.
    
    [N, M] = size(A);
    
    if N < 800
        
        % Just do it directly, its fast
        
        C = A\B;
        return;
        
    else
        
        % Do it in two steps.
        tol = 1e-9;
        C = A \ B;
        mask = any( abs(C) > tol, 2);
        
        C(~mask, :) = 0;
        
        N2 = sum(mask);
        M2 = round(N2*1.2);
        C(mask, :) = A(1:M2, mask) \ B(1:M2, :);
        
        return;
    end

end