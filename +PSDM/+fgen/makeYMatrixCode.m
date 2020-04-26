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
                c = c+1;
                vars.Up_ij{c} = Up_i(:, vars.Phi_acc_i_mask{i}(:, j));
                name = sprintf('Yacc_%d%d', i, j);
                [names.upsilon_acc_ij{c}, code.Up_acc{c}] = PSDM.fgen.codeSet(vars.Up_ij{c}, name, true, opt, ...
                                     vars.Up1, names.upsilon, [], opt);
                
            end

        end
        code.Yacc = strjoin(code.Up_acc, '\n');
        
        code.Y = strjoin({code.Yind, code.Yacc}, '\n\n');
        
    end
    
end
%{
function [vars, names, code] = makeYcode(E, name, vars, names, code, opt)
    % Makes the code for a set Y represented by E and with name "name".

    M = size(E, 2);
    DOF = vars.DOF;
    
    code.y = repelem({''}, M);
    for i = 1:M
        
        upsilon_i = vars.Up(:, i);
        a_i = vars.A(:, i);
        
        Up_ind = find( all( vars.Up1 == upsilon_i ), 1);
        A_ind = find( all( vars.A1 == a_i ), 1);
        
        Up_empty = all(upsilon_i == 0);
        A_empty = all(a_i == 0);
                
        % Make code for this element
        if ~A_empty && ~Up_empty
            
            names.y{i} = sprintf('%s(%d)', name, i);
            code.y{i} = sprintf('%s.*%s', names.a{A_ind}, names.upsilon{Up_ind});
            
        elseif ~A_empty && Up_empty
            
            names.y{i} = names.a{A_ind};
            code.y{i} = names.a{A_ind};
            
        elseif A_empty && ~Up_empty
            
            names.y{i} = names.upsilon{Up_ind};
            code.y{i} = names.upsilon{Up_ind};
            
        else
            error("This shouldn't happen");
        end
        
    end
    
    mask = ones(1, M, 'logical');
    code.Y = PSDM.fgen.assignVector( name, code.y(mask), opt );
    
    if strcmp(opt.tau_type, 'vector')
        
        for i = 1:DOF
            code.y_i{i} = PSDM.fgen.assignVector( sprintf( '%s_%d', name, i), names.y(vars.Phi_mask(:, i)), opt );
        end
        
        code.Y = sprintf( '%s\n\n%s\n%s', code.Y, strjoin(code.y_i, '\n'));
        
    end

end
%}