function x = mldivide2(A, b, niter, useSymbolic, exitTolerance, decompose)
    % MLDIVIDE2 Computes the least squares solution to A\b, however it
    % "recovers" the final digits of the computation using iteration with
    % newton's method and triple-precision residual calculations.
    %
    %   x = utils.mldivide2(A, b) Computes the least squares solution to
    %       A\b with 2 iterative improvement steps using triple precision.
    %
    %   x = utils.mldivide2(A, b, niter, useSymbolic, exit_tolerance)
    %       specifies the full algorithm options. 
    %       *   niter is the maximum number of iterations to do
    %           (default: 1). 
    %       *   useSymbolic specifies whether or not to use the symbolic
    %           toolbox for maximum precision in residual calculation. This
    %           produces very accurate results but is much slower and
    %           requires much more memory. Default: false.
    %       *   exit_tolerance: tolerance between improvement in steps
    %           before the algorithm auto-exits early.
    %
    % See also: utils.dot3p, utils.residual3p
    
    %% Parse Defaults
    if nargin < 3 || isempty(niter)
        niter = 1;
    end
    if nargin < 4 || isempty(useSymbolic)
        useSymbolic = false;
    end
    if nargin < 5 || isempty(exitTolerance)
        exitTolerance = eps(class(A));
    end
    if nargin < 6 || isempty(decompose)
        decompose = true;
    end
            
    tol2 = exitTolerance^2;
    
    % Decompose A
    if decompose
        [L, U, P] = lu(A);

        % Get standard solution using LUP triangle solving.
        x = U\(L\(P*b));
    else
        x = A\b;
    end
    
    norm_d_prev = inf(1, size(b, 2));
    
    for i = 1:niter
        
        % Get residual
        if useSymbolic && coder.target('matlab')
            r = double(A*sym(x,'f') - b);
        else
            r = utilities.residual3p(A, x, b);
        end
        
        % Get correction from LU decomposition
        if decompose
            d = U\(L\(P*r));
        else
            d = A\r;
        end
        
        % Make correction
        x = x - d;
        
        % Break?
        norm_d = utilities.norm2(d);
        if all(norm_d < tol2) || ...
           all(norm_d > norm_d_prev) || ...
           all(abs(norm_d - norm_d_prev) < exitTolerance)
            break;
        end
        
        % Store for future use
        norm_d_prev = norm_d;
        
    end

end