function [vars, names, code] = codeSetSingle(Su, name, keepAllElements, vars, names, code, opt)
    % Codes a single set
        
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
            names.s{i} = '1.0';
            
            if keepAllElements
                c = c+1;
                code.s{c} = '1.0';
            end
            
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
                
            elseif nnz(s) == 1
                
                j = find(s>0);
                names.s{i} = names.gamma{ s(j) }{ j };
                if keepAllElements
                    c = c+1;
                    code.s{c} = names.s{i};
                end
                
            end
            
        end
        
    end
    
    % Combine code for all elements
    if c > 0
        
        code.S = PSDM.fgen.assignVector( name, code.s(1:c), opt );
        
    else
        
        code.S = '';
        
    end

end