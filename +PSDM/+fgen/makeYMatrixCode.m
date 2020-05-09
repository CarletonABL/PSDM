function [vars, names, code] = makeYMatrixCode(vars, names, code, opt)
    % MAKEYMATRIXCODE Generates the code for defining Y in a function

    % loop and create code for elements of Y
    DOF = vars.DOF;
    
    if strcmp(opt.alg, 'ID')
        %% Inverse Dynamics
        
        % Generate set for Y
        keepAllElements = strcmp(opt.tau_type, 'matrix');
        [names.y, code.Y] = PSDM.fgen.codeSet(vars.E, 'Ypi', keepAllElements, opt, ...
                                 vars.Up1, names.upsilon, [], ...
                                 vars.A1, names.a, []);
        
        % Generate the sets for each vector column of tau
        if strcmp(opt.tau_type, 'vector')
        
            for i = 1:DOF
                code.y_i{i} = PSDM.fgen.assignVector( sprintf( 'Ypi_%d', i), names.y(vars.Phi_mask(:, i)), opt );
            end

            code.Y = sprintf( '%s\n\n%s\n%s', code.Y, strjoin(code.y_i, '\n'));

        end
        
    else
        %% Forward Dynamics
        
        % Make Y_induced set
        [names.yind, code.Yind] = PSDM.fgen.codeSet(vars.Eind, 'Yp_induced', false, opt, ...
                                 vars.Up1, names.upsilon, [], ...
                                 vars.A1, names.a, []);
                             
        % Generate the sets for each vector column of tau
        for i = 1:DOF
            code.yind_i{i} = PSDM.fgen.assignVector( sprintf( 'Yp_induced_%d', i), names.yind(vars.Phi_ind_mask(:, i)), opt );
        end
        code.Yind = sprintf( '%s\n\n%s\n%s', code.Yind, strjoin(code.yind_i, '\n'));
        
        % Now make Y_acc set for each row and column of mass matrix
        c = 0;
        for i = 1:DOF

            Up_i = vars.E(1:(3*DOF), vars.accelMask_i(i, :));
                        
            for j = 1:DOF

                % We will leverage the symmetry of D so no need to
                % calculate the bottom half of the matrix
                if i > j
                    continue;
                end

                % Calculate code
                Up_ij = Up_i(:, vars.Phi_acc_i_mask{i}(:, j));
                
                if ~isempty(Up_ij)
                    c = c+1;
                    vars.Up_ij{c} = Up_ij;
                    name = sprintf('Yacc_%d%d', i, j);
                    [names.upsilon_acc_ij{c}, code.Up_acc{c}] = PSDM.fgen.codeSet(vars.Up_ij{c}, name, true, opt, ...
                                         vars.Up1, names.upsilon, [], opt);
                end
                
            end

        end
        code.Yacc = strjoin(code.Up_acc, '\n');
        
        code.Y = strjoin({code.Yind, code.Yacc}, '\n\n');
        
    end
    
end