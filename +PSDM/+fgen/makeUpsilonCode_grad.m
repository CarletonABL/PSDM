function [vars, names, code] = makeUpsilonCode_grad(vars, names, code, opt)

    vars.DOF = size(vars.P, 3);

    % Break out some variables
    E = vars.E;
    vars.Up = E;
    Phi = vars.Phi;
    Phi_mask = vars.Phi_mask;
    Phi_mask_sum = cumsum(~Phi_mask(:));
    m = size(E, 2);
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
    for j = 1:DOF

        % Trim out rows with no contribution to this joint
        colMask = any(E > 0, 2);
        rowMask = Phi_mask(:, j);
        Su_t = E(colMask, rowMask);
        names_t.gamma{1} = names.gamma{1}(colMask);
        names_t.gamma{2} = names.gamma{2}(colMask);

        % Add names of running totals (Phi)
        names_t.y = cell(nnz(rowMask),1);
        c2 = 0;
        for i = 1:m
            if rowMask(i)
                c2 = c2+1;

                if opt.explicite_Phi
                    names_t.y{c2} = mat2str(Phi(i, j));
                else
                    Phi_ind = sub2ind(size(Phi), i, j);
                    Phi_ind_skip = Phi_ind - Phi_mask_sum(Phi_ind);
                    names_t.y{c2} = sprintf('Phi_b(%d)', Phi_ind_skip);
                end
            end
        end


        % Call genCode script on each joint
        code.tau_j{j} = PSDM.fgen.genCode(Su_t, sprintf('tau_%d', j), names_t, opt, 1);

        if strcmp( opt.gradual_joint_treatment, 'linear')
            code.tau_j{j} = sprintf('%s\n\ntau(%d, i) = tau_%d;\n', code.tau_j{j}, j, j);
        end

    end
    
    %% Generate code to combine tau
    
    if strcmp(opt.gradual_joint_treatment, 'parallel')
    
        % Make code for each case
        Phi_arg = '';
        if ~opt.explicite_Phi
            Phi_arg = ', Phi_b';
        end
        
        for j = 1:DOF
            cases{j} = sprintf('\t\t case %d\n\t\t\t tau_i(j) = joint%d(gi1, gi2%s);', ...
                j, j, Phi_arg);
        end
        
        % Make a parfor loop to evaluate each function
        code.tau = sprintf(['tau_i = coder.nullcopy(zeros(%d, 1));\n', ...
                            'parfor j = 1:%d\n', ...
                            '\tswitch j\n', ...
                            '%s\n', ...
                            '\tend\n', ...
                            'end\n', ...
                            'tau(:, i) = tau_i;'], ...
                            DOF, DOF, strjoin(cases, '\n'));
        
        % Package each joint function into a function and append to end of
        % function
        jointFunctions = cell(DOF,1);
        for j = 1:DOF
            jointFunctions{j} = sprintf( ...
                ['function tau_%d = joint%d(gi1, gi2%s)\n', ...
                 '%s \n', ...
                 'end'], j, j, Phi_arg, code.tau_j{j});
        end
        code.extra = strjoin(jointFunctions, '\n\n\n');
    
    elseif strcmp(opt.gradual_joint_treatment, 'linear')
        
        code.tau = strjoin(code.tau_j, '\n');
        
    end
   
end