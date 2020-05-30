function X = pinv2(A)
    % PINV2 Computes the pseudo-inverse of A, with extra-precise iterative
    % refinement. 
    %
    %   Ainv = utils.pinv2(A)
    %
    % See also: utils.linsolve.
    
    [U,S,V] = svd(A,'econ');
    s = diag(S);
    if nargin < 2 
        tol = max(size(A)) * eps(norm(s,inf));
    end
    r1 = nnz(s > tol);

    V1 = V(:, 1:r1);
    U1 = U(:, 1:r1);
    s1 = 1./s(1:r1);
    X = (V1.*s1.')*U1';
    
    % Solve for residual and fix estimate
    r = utilities.residual3p(A, X, eye(size(A, 1)));  
    d = A\r;
    
    % Correct
    X = X - d;

end