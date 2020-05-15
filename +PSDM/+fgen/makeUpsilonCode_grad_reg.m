function [vars, names, code] = makeUpsilonCode_grad_reg(vars, names, code, opt)

    vars.DOF = size(vars.P, 3);

    % Break out some variables
    E = vars.E;
    vars.Up = E;
    P = vars.P;
    P_mask = vars.P_mask;
    Phi = vars.Phi;
    Phi_mask = vars.Phi_mask;
    Phi_mask_sum = cumsum(~Phi_mask(:));
    m = size(E, 2);
    ell = size(P, 2);
    DOF = vars.DOF;
    
    % Allocate code not used
    code.Y = '';
    code.A = '';
    code.Up = '';
    code.extra = '';
        
    if opt.explicite_Phi
        code.Phi = '';
    end
    
    %% Loop through joints
    wasSkipped = zeros((ell*DOF), 1, 'logical');
    
    c1 = 0;
    c2 = 0;
    for k = 1:ell        
        for j = 1:DOF
            
            c1 = c1+1;

            % Trim out rows with no contribution to this joint
            colMask = any(E > 0, 2);
            rowMask = P_mask(:, k, j);
            
            if ~any(rowMask)
                wasSkipped(c1) = true;
                continue;
            end
            
            c2 = c2+1;
            
            Su_t = E(colMask, rowMask);
            names_t.gamma{1} = names.gamma{1}(colMask);
            names_t.gamma{2} = names.gamma{2}(colMask);

            % Add names of running totals (Phi)
            names_t.y = cell(nnz(rowMask),1);
            c3 = 0;
            for i = 1:m
                if rowMask(i)
                    c3 = c3+1;
                    names_t.y{c3} = mat2str(P(i, k, j));
                end
            end

            % Call genCode script on each joint
            code.tau_j{c2} = PSDM.fgen.genCode(Su_t, sprintf('Y_%d', c2), names_t, opt, 1);

            if strcmp( opt.gradual_joint_treatment, 'linear')
                code.tau_j{c2} = sprintf('%s\n\nY_i(%d) = Y_%d;\n', code.tau_j{c2}, sub2ind([DOF, ell], j, k), c2);
            end
            
        end
        
    end
    
    nFuncs = ell*DOF - sum(wasSkipped);
    
    %% Generate code to combine tau
    
    if strcmp(opt.gradual_joint_treatment, 'parallel')
            
        for c2 = 1:nFuncs
            cases{c2} = sprintf('\t\t case int16(%d)\n\t\t\t Y_i1(j) = joint%d(gi1, gi2);', ...
                c2, c2);
        end
        
        % Make a parfor loop to evaluate each function
        code.tau = sprintf(['Y_i1 = coder.nullcopy(zeros(%d, 1));\n', ...
                            'parfor j = int16(1):int16(%d)\n', ...
                            '\tswitch j\n', ...
                            '%s\n', ...
                            '\tend\n', ...
                            'end\n', ...
                            'Y_i = zeros(%d, 1);\n', ...
                            'Y_i(nzeroY) = Y_i1;\n'], ...
                            nFuncs, nFuncs, strjoin(cases, '\n'), ell*DOF);
        
        % Package each joint function into a function and append to end of
        % function
        jointFunctions = cell(DOF,1);
        for c2 = 1:nFuncs
            jointFunctions{c2} = sprintf( ...
                ['function Y_%d = joint%d(gi1, gi2)\n', ...
                 '%s \n', ...
                 'end'], c2, c2, regexprep(code.tau_j{c2}, '\n+', '\n'));
        end
        code.extra = strjoin(jointFunctions, '\n\n\n');
        
        code.setup1 = sprintf('%s\nnzeroY = coder.const(%s);\n', ...
            code.setup1, mat2str(~wasSkipped));
    
    elseif strcmp(opt.gradual_joint_treatment, 'linear')
        
        code.tau = sprintf('Y_i = zeros(%d, 1); \n %s', ...
            ell*DOF, strjoin(regexprep(code.tau_j, '\n+', '\n'), '\n'));
        
    end
    
    % Append final multiplication
    code.tau = sprintf('%s \ntau(:, i) = reshape(Y_i, [%d, %d]) * Theta;', code.tau, DOF, ell);
   
end