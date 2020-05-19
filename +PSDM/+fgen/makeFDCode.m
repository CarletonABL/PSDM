function [vars, names, code] = makeFDCode(vars, names, code, opt)

    % Break out some variables
    DOF = vars.DOF;

    %% Step 1: Make induced torque code
    if opt.return_all
        tauIndName = arrayfun(@(i) sprintf('tauInd[startInd + %d]', i), 0:DOF-1, 'UniformOutput', false);
    else
        tauIndName = arrayfun(@(i) sprintf('tauInd[%d]', i), 0:DOF-1, 'UniformOutput', false);
    end
    
    [tauIndCode, names, code] = PSDM.fgen.makeTauCode(vars.E_induced, vars.P_induced, tauIndName, 'Y_ind', false, names, code, opt);
    code.tau_ind = tauIndCode;
    
    %% Step 2: Make code for each element of D
    names_acc = names;
    names_acc.gamma{1} = names.gamma{1}(1:(3*DOF));
    names_acc.gamma{2} = names.gamma{2}(1:(3*DOF));
    
    code.D = cell(DOF^2, 1);
    c = 0;
    for j = 1:DOF
        for i = 1:DOF
            % Only do upper triangle of matrix
            if i > j
                continue;
            end
            c = c+1;
            E_ij = vars.E_accel{j};
            P_ij = vars.P_accel{j}(:, :, i);
            
            Dname = sprintf('d[%d]', c-1);
            [Di_code, names, code] = PSDM.fgen.makeTauCode(E_ij, P_ij, ...
                    {Dname}, sprintf('Y_D%d%d', i, j), ...
                    false, names, code, opt);
            code.D{c} = Di_code;
            
        end
    end
    
    code.tau = sprintf('%s\n\n%s', code.tau_ind, strjoin(code.D(1:c), '\n\n'));
   
end