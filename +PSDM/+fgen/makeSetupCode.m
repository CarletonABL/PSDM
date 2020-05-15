function [vars, names, code] = makeSetupCode(E, P, Theta, opt)
    % Make the base code to generate the elements of A and Upsilon, which
    % is used in both the makeForwardDynamics and makeInverseDynamics
    % functions.
    
    vars.E = E;
    vars.P = P;
    vars.Theta = Theta;
    vars.DOF = size(P, 3);
    DOF = vars.DOF;
    
    if strcmp(opt.alg, 'ID')
        colMask = any(E > 0, 2);
    else
        colMask = any(E(1:(4*DOF), :) > 0, 2);
    end
    squareMask = any( E(colMask, :) > 1, 2);
    vars.colMask = colMask;
    vars.squareMask = squareMask;
    
    %% Make a list of upsilon vectors from input
    
    code.setup1 = '';
                     
    n = size(E, 1);
    ind = 1:n;
    ind1 = ind(colMask);
    ind2 = ind1(squareMask);
    
    % Generate gamma and gamma squared terms
    names.gamma{1} = repelem({''}, n);
    names.gamma{2} = repelem({''}, n);
    for i = 1:nnz(colMask)
        names.gamma{1}{ ind1(i) } = sprintf('gi1(%d)', i);
        code.gamma{1}{ i } = sprintf('gi(%d)', ind1(i));
    end
    for i = 1:nnz(squareMask)
        names.gamma{2}{ ind2(i) } = sprintf('gi2(%d)', i);
        code.gamma{2}{ i } = sprintf('gi(%d)^2', ind2(i));
    end
    
    code.gamma1 = PSDM.fgen.assignVector( 'gi1', code.gamma{1}, opt );
    code.gamma2 = PSDM.fgen.assignVector( 'gi2', code.gamma{2}, opt );
    code.setup2 = strjoin( {code.gamma1, code.gamma2}, '\n' );

    
    if strcmp(opt.alg, 'FD')
         % Need to extract the acceleration terms from E, into a mask
        vars.accelMask_i = E((1:vars.DOF)+4*vars.DOF, :) > 0;
        vars.accelMask = any(vars.accelMask_i, 1);
        vars.Eind = E( :, ~ vars.accelMask);
    end
    
    
        
end