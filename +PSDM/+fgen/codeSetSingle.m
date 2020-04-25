function [vars, names, code] = codeSetSingle(Su, name, vars, names, code, opt)
    % Codes a single set
    
    mult_type = opt.mult_type;
    
    Mu = size(Su, 2);
    
    % Pre-init elements
    code.s = repelem({''}, Mu);
    names.s = repelem({''}, Mu);
    
    % Loop through each element of unique set
    c = 0;
    for i = 1:Mu
        
        % Break out functions
        s = Su(:, i);
        
        % Find name for element 1
        if all(s == 0)
            
            % Function is one, ignore it
            names.s{i} = '1';
            
        else
                        
            % Make a list of all individual functions in this product (1 or
            % 2). names.gamma should be a cell structure of the elements of
            % gamma which apply only to this set.
            
            if nnz(s) > 1
                % Need to define a new operation
                c = c + 1;
                names.s{i} = sprintf('%s(%d)', name, c);
                str = repelem({''}, nnz(s) );
                d = 1;
                for j = 1:numel(s)
                    if s(j) == 0
                        continue;
                    end
                    str{d} = names.gamma{ s(j) }{ j };
                    d = d+1;
                end

                % Combine
                code.s{c} = sprintf('%s', strjoin(str, ".*"));
                code.s_els1{c} = str{1};
                code.s_els2{c} = str{2};
                
            elseif nnz(s) == 1
                j = find(s>0);
                names.s{i} = names.gamma{ s(j) }{ j };
            end
            
        end
        
    end
    
    % Combine code for all elements
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

end