function [vars, names, code] = makeTauCode(vars, names, code, opt)

    % Otherwise, do the individual one
    Phi = vars.Phi;
    E = vars.E;
    DOF = vars.DOF;
    p = size(E, 2);

    tau_type = opt.tau_type;
    
    if strcmp(tau_type, 'matrix')
        code.tau = 'tau(:, i) = (Ypi * Phi_b)'';';
        return;
    end
    
    if strcmp(tau_type, 'vector')
    
        for i = 1:DOF
            code.Phi_i{i} = sprintf('Phi_b_%d = coder.const(%s);', i, mat2str(Phi(vars.Phi_mask(:, i), i)));
            code.tau_i{i} = sprintf('tau(%d, i) = Ypi_%d * Phi_b_%d;', i, i, i);
        end
        
        code.Phi = strjoin(code.Phi_i, '\n');
        code.tau = strjoin(code.tau_i, '\n');
        
        return;
        
    end
    
    
    tol = 1e-10;
    
    for i = 1:DOF
        
        code.tau_ij = cell(0,0);
        c = 0;
        % Loop through all terms
        for j = 1:p
            phi_ji = Phi(j, i);
            
            if abs(phi_ji) < tol
                % Term is not significant, ignore
                continue;
            elseif abs( abs(phi_ji) - 1) < tol
                msign = {'', '-'};
                c = c+1;
                code.tau_ij{c} = sprintf('%s%s', msign{ (phi_ji<0) + 1 }, names.y{j});
            else
                c = c+1;
                code.tau_ij{c} = sprintf('%s.*%s', mat2str(phi_ji), names.y{j});
            end
            
        end
        
        % Concatenate
        code.tau_ij_sum{i} = PSDM.fgen.assignVector( sprintf('tau_%dj', i), code.tau_ij(1:c), opt );
        %code.tau_ij_sum{i} = sprintf('tau_%dj = [%s];\n', ...
        %                        i, strjoin(code.tau_ij(1:c), ','));
        code.tau_i{i} = sprintf('tau(%d, i) = sum(tau_%dj, 2);', ...
                                i, i);
        
    end
    
    % Concatenate joint torques
    code.Phi = '';
    code.tau = sprintf('%s\n\n%s', strjoin(code.tau_ij_sum, '\n'), strjoin(code.tau_i, '\n'));

end