function [vars, names, code] = makeSetupCode(E, P, opt)
    % Make the base code to generate the elements of A and Upsilon, which
    % is used in both the makeForwardDynamics and makeInverseDynamics
    % functions.
    
    % Break out some variables
    vars.E = E;
    vars.P = P;
    vars.DOF = size(P, 3);
    DOF = vars.DOF;
    vars.ell = size(P, 2);
    ell = vars.ell;
    
    % Eliminate columns of E which are not used
    if strcmp(opt.alg, 'ID')
        colMask = any(E > 0, 2);
    else
        colMask = any(E(1:(4*DOF), :) > 0, 2);
    end
    squareMask = any( E(colMask, :) > 1, 2);
    vars.colMask = colMask;
    vars.squareMask = squareMask;
    
    %% Pre-set other optional code
    code.setup1 = '';
    code.setup2 = '';
    code.extra = '';
    code.tau = '';
    code.name_def = '';
    
    code.all = cell(0, 1);
    names.all = cell(0, 1);
    
    names.def = cell(0, 1);
    
    %% Make a list of gamma vectors from input
    
    % some setup
    n = size(E, 1);
    ind = 1:n;
    ind1 = ind(colMask);
    ind2 = ind1(squareMask);
    
    % Generate gamma and gamma squared terms
    names.gamma{1} = repelem({''}, n);
    names.gamma{2} = repelem({''}, n);
    
    % Assign unsquared values
    for i = 1:nnz(colMask)
        names.gamma{1}{ ind1(i) } = sprintf('gi[%d]', ind1(i)-1);
    end
    
    code.gamma{2} = cell(nnz(squareMask), 1);
    for i = 1:nnz(squareMask)
        names.gamma{2}{ ind2(i) } = sprintf('gi2_p%d', i);
        code.gamma{2}{ i } = sprintf('gi[%d]*gi[%d]', ind2(i)-1, ind2(i)-1);
    end
    
    % Assign variables
    [code.gamma2, names] = PSDM.fgen.assignVector( 'gi2', code.gamma{2}, names, opt );
    code.setup2 = code.gamma2;

    % Some extra processing if we're doing forward dynamics
    if strcmp(opt.alg, 'FD')
         % Need to extract the acceleration terms from E, into a mask
        vars.accelMask_i = E((1:vars.DOF)+4*vars.DOF, :) > 0;
        vars.accelMask = any(vars.accelMask_i, 1);
        vars.Eind = E( :, ~ vars.accelMask);
    end
        
end