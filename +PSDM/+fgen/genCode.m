function [setCode, names, code] = genCode(S, name, name_prefix, names_t, names, code, opt, depth)
    % GENCODE given a set S, generates code for this set by gradually
    % multiplying in P row by row.
    %
    % INPUTS: 
    %   - S: The set, in E notation
    %   - name: The name of the final variable.
    %   - name_prefix: The prefix to give to intermediate variables.
    %   - names_t: The "trimmed" set of names to use in codegen.
    %   - names: the full names object
    %   - code: The full code object
    %   - opt: The options object.
    %   - depth: The current recursion depth
    %
    % OUTPUTS:
    %   - setCode: The code for the current set.
    %   - names, code: The updated code and names objects.

    % Break out variables
    n = size(S, 1);
    m = size(S, 2);
    
    % If empty set, return zero
    if m < 1
        setCode = sprintf('\t\t%s = 0;\n', name);
        names = PSDM.fgen.addToNames(name, names);
        return;
    end
    if n < 1
        % Just need to sum all the P's
        Psum = sum(cellfun(@str2num, names_t.y));
        setCode = sprintf('%s = %s;\n', name, mat2str(Psum));
        names = PSDM.fgen.addToNames(name, names);
        return;
    end

    % Take out first row
    s = 1;
    
    % Define subsets
    S1 = S(1:s, :);
    S2 = S(s+1:end, :);

    % Reduce bottom set to just unique values
    Su1 = unique(S1', 'rows', 'stable')';
    Su2 = unique(S2', 'rows', 'stable')';
    
    % Get sizes of subsets
    m1 = size(Su1, 2);
    m2 = size(Su2, 2);
    
    % Is this the final iteration? If so, name of returned variable
    % changes.
    if n == 1 || all(all( Su2 == 0, 1), 2)
        name_i = name;
        final = true;
    else
        name_i = sprintf('%s_d%d', name_prefix, depth);
        final = false;
    end
    
    % Reduce and condense in loop. Loop through each element of larger set
    % and make code for all elements of first set
    c = 0;
    yCode = cell(0,0);
    yNames = cell(m2, 1);
    codeNameInd = zeros(m2, 1);
    for i = 1:m2
        
        % Get the element of second set
        s2 = Su2(:, i);
        
        % Find the indices corresponding elements of first set
        s1_ind = find( all( S2 == s2, 1) );
        m1i = numel(s1_ind);
        
        % Find the actual elements of the first set
        s1 = S1( :, s1_ind );

        % Is the first set singular and/or empty?
        s1_empty = all(s1 == 0, 1);
        s1_single = m1i == 1;
        
        % Make code for a new y variable
        if ~s1_single || any(~s1_empty) || final
            % Set is not singular, and at least one is non-empty. Need to
            % make a new interim variable
            
            % Set forwarding flag to true. If any of the below instructions
            % sets it to false, then we can't forward along the variable
            forwardFlag = true;
            
            % Preallocate
            terms = cell(m1i,1);
            
            % Loop through each term in s1
            for k = 1:m1i
                if s1_empty(k)
                    % If its empty, just use its name in the formula
                    terms{k} = sprintf('%s', names_t.y{s1_ind(k)});
                else
                    % Otherwise, we need to multiply with the correct value
                    % of P. However, we also avoid doing that if the value
                    % is +- 1.
                    switch names_t.y{s1_ind(k)}
                        case '1'
                            terms{k} = sprintf('%s', names_t.gamma{ s1(1,k) }{ 1 });
                        case '-1'
                            forwardFlag = false;
                            terms{k} = sprintf('-%s', names_t.gamma{ s1(1,k) }{ 1 });
                        otherwise
                            forwardFlag = false;
                            terms{k} = sprintf('%s*%s', ...
                                names_t.y{s1_ind(k)}, names_t.gamma{ s1(1,k) }{ 1 });
                    end
                end
            end
            
            if m1i == 1 && forwardFlag && ~final
                % We now have a list of all the terms. If forward flag is true
                % and this is not the final iteration, just forward along the
                % name.
                yNames{i} = terms{1};
            else
                % Concatenate terms into a single addition
                code_i = strrep( strjoin(terms, '+'), '+-', '-');

                % Check if this set of terms has been already calculated
                name_already = PSDM.fgen.findEquivNames(code_i, names, code);
                
                if isempty(name_already)
                    % New term, define it
                    c = c+1;
                    yCode{c} = code_i;
                    codeNameInd(c) = i;
                    if final
                        yNames{i} = name_i;
                    else
                        yNames{i} = sprintf('%s_p%d', name_i, c);
                    end
                    
                    % Add to list of all terms
                    [code, names] = addNameCode(code_i, yNames{i}, code, names);
                    
                else
                    
                    % Just forward along the name
                    yNames{i} = name_already;
                    if final
                        % Need to write out an assignment here
                        c = c+1;
                        yCode{c} = yNames{i};
                    end
                    
                end
            end
            
        else
            % s1 is single and empty, so just forward along the name
            yNames{i} = names_t.y{s1_ind(1)};
            
        end
                
    end
    
    % Make code
    if c>0
        if final
            setCode = sprintf('%s = %s;', name_i, yCode{1} );
            names = PSDM.fgen.addToNames(name_i, names);
        else
            [setCode, names] = PSDM.fgen.assignVector( name_i, yCode, names, opt );
        end
    else
        setCode = '';
    end
    
    % Call recursively
    if ~ final
        % Define variables for recursion
        names_s = names_t;
        names_s.y = yNames;

        names_s.gamma{1} = names_t.gamma{1}(2:end);
        names_s.gamma{2} = names_t.gamma{2}(2:end);
        
        % Call recursively
        [setCode_s, names, code] = PSDM.fgen.genCode(Su2, name, name_prefix, names_s, names, code, opt, depth+1);
        
        % Combine code
        setCode = sprintf('%s\n\n\t\t%s', setCode, setCode_s);
        
    end
    
end

%% HELPER FUNCTIONS

function [code, names] = addNameCode(codeAdd, nameAdd, code, names)
    N = size(code.all, 1);
    code.all{N+1, 1} = codeAdd;
    names.all{N+1, 1} = nameAdd;
end