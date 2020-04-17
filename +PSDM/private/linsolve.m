function x = linsolve(A, b, decompose)
    % LINSOLVE Solves a set of linear equations. Uses A\b if
    % c.use_iterative_refinement = false, otherwise uses
    % utilities.mldivide2 and gets higher accuracy (runs slower though).
    
    if nargin < 3 || isempty(decompose)
        decompose = true;
    end
    
    coder.extrinsic('utilities.mldivide2');

    c = PSDM.config;
    if ~c.use_iterative_refinement
        x = A\b;
    else
        x = coder.nullcopy(zeros(size(A, 2), size(b, 2)));
        x = utilities.mldivide2(A, b, 1, false, eps, decompose);
    end
    
end