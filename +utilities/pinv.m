function X = pinv(A, varargin)
    % PINV Computes the pseudo-inverse of A, with extra-precise iterative
    % refinement. Otherwise takes all the same arguments as the Pinv
    % function.
    %
    %   X = utilities.pinv(A)
    %   X = utilities.pinv(A, tol)
    %
    % See also: utilities.linsolve, pinv

    % Calculate pinv normally
    X = pinv(A, varargin{:});
    
    % Solve for residual in triple precision and fix estimate
    r = utilities.residual3p(A, X, eye(size(A, 1)));  
    
    % Correct
    X = X - X*r;

end