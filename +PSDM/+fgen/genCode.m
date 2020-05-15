function [setCode] = genCode(S, name, names, opt, depth, keepAllElements)

    if nargin < 6
        keepAllElements = false;
    end

    n = size(S, 1);
    m = size(S, 2);
    % If empty set, return zero
    if m < 1
        setCode = sprintf('%s = 0;\n', name');
        return;
    end

    % Take out first row
    s = 1;
    
    S1 = S(1:s, :);
    S2 = S(s+1:end, :);

    % Reduce bottom set to just unique values
    Su1 = unique(S1', 'rows', 'stable')';
    Su2 = unique(S2', 'rows', 'stable')';
    
    m1 = size(Su1, 2);
    m2 = size(Su2, 2);
    
    % Condense & reduce
    if n == 1 || all( Su2 == 0, 'all')
        name_i = name;
        final = true;
    else
        name_i = sprintf('%s_%dp', name, depth);
        final = false;
    end
    
    c = 0;
    yCode = cell(0,0);
    yNames = cell(m2, 1);
    for i = 1:m2
        
        s2 = Su2(:, i);
        s1_ind = find( all( S2 == s2, 1) );
        
        
        s1 = S1( :, s1_ind );

        s1_empty = all(s1 == 0, 1);
        m1i = size(s1, 2);
        s1_single = m1i == 1;
        
        % Make code for a new y variable
        if ~s1_single || any(~s1_empty)
            
            forwardFlag = true;
            terms = cell(m1i,1);
            for k = 1:m1i
                if s1_empty(k)
                    terms{k} = sprintf('%s', names.y{s1_ind(k)});
                else
                    % handle some special cases here
                    switch names.y{s1_ind(k)}
                        case '1'
                            terms{k} = sprintf('%s', names.gamma{ s1(1,k) }{ 1 });
                        case '-1'
                            forwardFlag = false;
                            terms{k} = sprintf('-%s', names.gamma{ s1(1,k) }{ 1 });
                        otherwise
                            forwardFlag = false;
                            terms{k} = sprintf('%s.*%s', names.gamma{ s1(1,k) }{ 1 }, names.y{s1_ind(k)});
                    end
                end
            end
            
            if m1i == 1 && forwardFlag && ~final
                yNames{i} = terms{1};
            else
                c = c+1;
                yCode{c} = strjoin(terms, '+');
                yNames{i} = sprintf('%s(%d)', name_i, c);
            end
            
        else
            % s1 is single and empty, so just forward along the name
            yNames{i} = names.y{s1_ind(1)};
        end
                
    end
    
    % Make code
    if c>0
        setCode = PSDM.fgen.assignVector( name_i, yCode, opt );
    elseif keepAllElements
        %setCode = sprintf('%s = name_i)
    else
        setCode = '';
    end
    names_s = names;
    names_s.y = yNames;
    
    names_s.gamma{1} = names.gamma{1}(2:end);
    names_s.gamma{2} = names.gamma{2}(2:end);
    
    % Call recursively
    if ~ final
        [setCode_s] = PSDM.fgen.genCode(Su2, name, names_s, opt, depth+1, false);
        setCode = sprintf('%s\n\n%s', setCode, setCode_s);
    else
        % If get to bottom of tree and have been just forwarding names,
        % need to break them out now
        %         if isempty(setCode) && false
        %             setCode = sprintf('%s = %s;\n', name_i, yNames{1});
        %         end
    end
    
end

%% HELPER FUNCTIONS

function s1_ind_min = findS1Indices(s1, Su1)
    
    s1_ind_min = zeros(1, size(s1, 2));
    for i = 1:size(s1, 2)
        s1_ind_min(i) = find( all( Su1 == s1(:, i), 1), 1 );
    end

end