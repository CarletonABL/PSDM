function x = linsolve(A, b)
    % LINSOLVE Solves a set of linear equations. Uses A\b if
    % c.use_iterative_refinement = false, otherwise uses
    % utilities.mldivide2 and gets higher accuracy (runs slower though).
        
    coder.extrinsic('utils.mldivide2');

    c = PSDM.config;
    if ~c.use_iterative_refinement
        x = A\b;
    else
        x = coder.nullcopy(zeros(size(A, 2), size(b, 2)));
        x = utilities.mldivide2(A, b);
    end
    
end