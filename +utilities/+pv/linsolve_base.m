function x = linsolve_base(A, b, niter, useSymbolic, exitTolerance)

    % Define some variables
    tol2 = exitTolerance^2;
    
    % Decompose A so that we don't have to resolve everything every time
    dA = decomposition(A, 'CheckCondition', false);
    
    % Get an initial solution
    x = dA\b;
    
    norm_d_prev = inf(1, size(b, 2));
    
    for i = 1:niter
        
        % Get residual
        if useSymbolic && coder.target('matlab')
            r = double(A*sym(x,'f') - b);
        else
            r = utilities.residual3p(A, x, b);
        end
        
        d = dA \ r;
        
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