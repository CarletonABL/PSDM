function [vars, names, code] = makeSetupCode(E, P, Theta)
    % Make the base code to generate the elements of A and Upsilon, which
    % is used in both the makeForwardDynamics and makeInverseDynamics
    % functions.
    
    vars.E = E;
    vars.P = P;
    vars.Theta = Theta;
    vars.DOF = size(P, 3);
    
    colMask = any(E > 0, 2);
    squareMask = any( E(colMask, :) > 1, 2);
    vars.colMask = colMask;
    vars.squareMask = squareMask;
    
    %% Make a list of upsilon vectors from input
    
    code.setup = sprintf(['colMask = coder.const(%s);\n', ...
                         'squareMask = coder.const(%s);\n'],...
                         mat2str(colMask), mat2str(squareMask));
    if opt.use_gpu
        code.setup = sprintf('coder.gpu.kernelfun();\n%s', coder.setup);
    end
                     
    n = size(E, 1);
    ind = 1:n;
    ind1 = ind(colMask);
    ind2 = ind1(squareMask);
    
    names.gamma{1} = repelem({''}, n);
    names.gamma{2} = repelem({''}, n);
    for i = 1:nnz(colMask)
        names.gamma{1}{ ind1(i) } = sprintf('gi1(%d)', i);
    end
    for i = 1:nnz(squareMask)
        names.gamma{2}{ ind2(i) } = sprintf('gi2(%d)', i);
    end
        
end