function [vars, names, code] = codeSet(Su, name, ...
                                 vars, names, code, ...
                                 vars1, names1, code1, ...
                                 vars2, names2, code2, opt)
    % Generates the actual code for a set.
    
    mult_type = opt.mult_type; %'matrix';
    
    % Get size of set
    Mu = size(Su, 2);
    
    % Break out some variables
    S1 = vars.Su1;
    S2 = vars.Su2;
    Su1 = vars1.Su;
    Su2 = vars2.Su;
    
    % Pre-init elements
    code.s = repelem({''}, Mu);
    names.s = repelem({''}, Mu);
    
    % Loop through each element of unique set
    c = 0;
    for i = 1:Mu
        
        % Break out functions
        s1 = S1(:, i);
        s2 = S2(:, i);
        
        % Find name for element 1
        s1_ind = find( all( Su1 == s1, 1 ), 1);
        s1_empty = all(s1 == 0);
        
        % Find name for element 2
        s2_ind = find( all( Su2 == s2, 1 ), 1);
        s2_empty = all(s2 == 0);
        
        % Make code for this element
        if ~s1_empty && ~s2_empty
            c = c+1;
            names.s{i} = sprintf('%s(%d)', name, c);
            code.s{c} = sprintf('%s.*%s', names1.s{s1_ind}, names2.s{s2_ind});
            code.s_els1{c} = names1.s{s1_ind};
            code.s_els2{c} = names2.s{s2_ind};
        elseif ~s1_empty && s2_empty
            names.s{i} = names1.s{s1_ind};
        elseif s1_empty && ~s2_empty
            names.s{i} = names2.s{s2_ind};
        else
            names.s{i} = names2.s{s2_ind};
        end
        
    end
    
    % Now, concatenate everything together.
    if c > 0
        if strcmp(mult_type, 'individual')
            % code.S = sprintf('%s = [%s];', name, strjoin(code.s(1:c), ','));
            code.S = PSDM.fgen.assignVector( name, code.s(1:c), opt );
        else
            v1 = PSDM.fgen.assignVector( strcat(name, '_m1'), code.s_els1(1:c), opt );
            v2 = PSDM.fgen.assignVector( strcat(name, '_m2'), code.s_els2(1:c), opt );
            % v1 = sprintf('%s = [%s];', strcat(name, '_m1'), strjoin(code.s_els1(1:c), ','));
            % v2 = sprintf('%s = [%s];', strcat(name, '_m2'), strjoin(code.s_els2(1:c), ','));
            v3 = sprintf('%s = %s .* %s;', name, strcat(name, '_m1'), strcat(name, '_m2'));
            code.S = strjoin({v1, v2, v3}, '\n');
        end
    else
        code.S = '';
    end
    
    if ~ isempty(code1.S)
        code.S = sprintf('%s\n\n%s', code1.S, code.S);
    end
    
    if ~ isempty(code2.S)
        code.S = sprintf('%s\n\n%s', code2.S, code.S);
    end
    
    names.S = name;
    
end