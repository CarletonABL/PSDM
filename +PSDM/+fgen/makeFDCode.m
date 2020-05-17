function [vars, names, code] = makeFDCode(vars, names, code, opt)

    % Break out some variables
    DOF = vars.DOF;

    %% Step 1: Make induced torque code
    code.tau_ind = PSDM.fgen.makeYbCode(vars.E_induced, vars.P_induced, 'tau_ind', 'Y_ind', names, opt);
    
    %% Step 2: Make code for each element of D
    names_acc = names;
    names_acc.gamma{1} = names.gamma{1}(1:(3*DOF));
    names_acc.gamma{2} = names.gamma{2}(1:(3*DOF));
    
    code.D = cell(DOF^2, 1);
    c = 0;
    for i = 1:DOF
        for j = 1:DOF
            % Only do upper triangle of matrix
            if i > j
                continue;
            end
            c = c+1;
            E_ij = vars.E_accel{j};
            P_ij = vars.P_accel{j}(:, :, i);
            code.D{c} = PSDM.fgen.makeYbCode(E_ij, P_ij, ...
                    sprintf('D(%d, %d)', i, j),...
                    sprintf('Y_D%d%d', i, j), ...
                    names, opt);
            code.D{c} = sprintf('%s\nD(%d, %d) = D(%d, %d);\n', ...
                code.D{c}, j, i, i, j);
            
        end
    end
    
    code.tau = sprintf('%s\n\n%s', code.tau_ind, strjoin(code.D(1:c), '\n'));
   
end