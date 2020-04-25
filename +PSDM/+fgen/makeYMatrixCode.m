function [vars, names, code] = makeYMatrixCode(vars, names, code, name, opt)
    % MAKEYMATRIXCODE Generates the code for defining Y in a function

    mult_type = opt.mult_type;
    tau_type = opt.tau_type;
    
    Phi = vars.Phi;
    tol = 1e-10;

    % loop and create code for elements of Y
    M = size(vars.E, 2);
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
            
            code.y_els1{i} = names.a{A_ind};
            code.y_els2{i} = names.upsilon{Up_ind};
            
        elseif ~A_empty && Up_empty
            
            names.y{i} = names.a{A_ind};
            code.y{i} = names.a{A_ind};
            
            code.y_els1{i} = names.a{A_ind};
            code.y_els2{i} = '1.0';
            
        elseif A_empty && ~Up_empty
            
            names.y{i} = names.upsilon{Up_ind};
            code.y{i} = names.upsilon{Up_ind};
            
            code.y_els1{i} = '1.0';
            code.y_els2{i} = names.upsilon{Up_ind};
            
        else
            error("This shouldn't happen");
        end
        
    end
    
    mask = ones(1, M, 'logical');
    code.Y = makeYcode(code, name, mult_type, mask, opt);
    
    if strcmp(tau_type, 'vector')
        
        mask = abs(Phi) > tol;
        vars.Phi_mask = mask;
        for i = 1:DOF
            code.y_i{i} = PSDM.fgen.assignVector( sprintf( '%s_%d', name, i), names.y(mask(:, i)), opt );
            % sprintf('%s_%d = [%s];', name, i, strjoin( names.y(mask(:, i)), ',' ));
        end
        
        code.Y = sprintf( '%s\n\n%s\n%s', code.Y, strjoin(code.y_i, '\n'));
        
    end
    
end

function codeY = makeYcode(code, name, type, mask, opt)

    % Concatenate and save
    if strcmp(type, 'individual')
        % codeY = sprintf('%s = [%s];', name, strjoin(code.y(mask), ','));
        codeY = PSDM.fgen.assignVector( name, code.y(mask), opt );
    else
        v1 = PSDM.fgen.assignVector( strcat(name, '_m1'), code.y_els1(mask), opt );
        v2 = PSDM.fgen.assignVector( strcat(name, '_m2'), code.y_els2(mask), opt );
        % v1 = sprintf('%s = [%s];', strcat(name, '_m1'), strjoin(code.y_els1(mask), ','));
        % v2 = sprintf('%s = [%s];', strcat(name, '_m2'), strjoin(code.y_els2(mask), ','));
        v3 = sprintf('%s = %s .* %s;', name, strcat(name, '_m1'), strcat(name, '_m2'));
        codeY = strjoin({v1, v2, v3}, '\n');
    end
end