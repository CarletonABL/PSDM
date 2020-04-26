function [vars, names, code] = makeTauCode(vars, names, code, opt)

    DOF = vars.DOF;

    if strcmp(opt.alg, 'ID')
        %% INVERSE DYNAMICS
        if strcmp(opt.tau_type, 'matrix')

            code.tau = 'tau(:, i) = (Ypi * Phi_b)'';';

        else

            for i = 1:DOF
                code.tau_i{i} = sprintf('tau(%d, i) = Ypi_%d * Phi_b_%d;', i, i, i);
            end

            code.tau = strjoin(code.tau_i, '\n');

        end
        
    else
        %% FORWARD DYNAMICS
        
        % First, generate the induced torques
        for i = 1:DOF
            code.tau_ind_i{i} = sprintf('tau_ind(%d) = Yp_induced_%d * Phi_ind_%d;', i, i, i);
        end
        code.tau_ind = strjoin(code.tau_ind_i, '\n');
        
        % Now, generate D
        c=0;
        for i = 1:DOF
            for j = 1:DOF
                % We will leverage the symmetry of D so no need to
                % calculate the bottom half of the matrix
                if i > j
                    continue;
                end
                c = c+1;
                code.D_ij{c} = sprintf('D(%d, %d) = Yacc_%d%d * Phi_acc_%d%d;', ...
                                       i, j, i, j, i, j);
                % Assign the other half now too
                if i ~= j
                    code.D_ij{c} = sprintf('%s\nD(%d, %d) = D(%d, %d);', ...
                                           code.D_ij{c}, j, i, i, j);
                end
            end
        end
        
        code.D = strjoin(code.D_ij, '\n');
                     
        code.tau = strjoin({code.tau_ind, code.D}, '\n\n');
                           
    end
    
end