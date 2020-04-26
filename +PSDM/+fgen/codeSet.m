function [setNames, setCode] = codeSet(Su, name, keepAllElements, opt, ...
                                 Su1, set1Names, set1Code, ...
                                 Su2, set2Names, set2Code)
    % Generates the actual code for a set.

    % Get size of set
    Mu = size(Su, 2);
    
    % Some defaults. If only one set is given, then we just want to
    % "rearrange" that set's names to form the desired variables
    if nargin < 10
        % No second set given
        % Can trick the following set by just given a bunch of zeros
        % as second set
        Su = vertcat(Su, zeros(1, Mu));
        Su2 = 0;
    end
    
    % Break out some variables
    n1 = size(Su1, 1);
    n2 = size(Su2, 1);
    S1 = Su(1:n1, :);
    S2 = Su((n1+1):(n1+n2), :);
    
    % Pre-init elements
    elCode = repelem({''}, Mu);
    setNames = repelem({''}, Mu);
    
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
            setNames{i} = sprintf('%s(%d)', name, c);
            elCode{c} = sprintf('%s.*%s', set1Names{s1_ind}, set2Names{s2_ind});
        elseif ~s1_empty && s2_empty
            setNames{i} = set1Names{s1_ind};
            if keepAllElements
                c = c+1;
                elCode{c} = setNames{i};
            end
        elseif s1_empty && ~s2_empty
            setNames{i} = set2Names{s2_ind};
            if keepAllElements
                c = c+1;
                elCode{c} = setNames{i};
            end
        else
            setNames{i} = '1.0';
            if keepAllElements
                c = c+1;
                elCode{c} = setNames{i};
            end
        end
        
    end
    
    % Now, concatenate everything together.
    if c > 0
        setCode = PSDM.fgen.assignVector( name, elCode(1:c), opt );
    else
        setCode = '';
    end
    
    if exist('set1Code', 'var') &&  ~ isempty(set1Code)
        setCode = sprintf('%s\n\n%s', set1Code, setCode);
    end
    
    if exist('set2Code', 'var') && ~ isempty(set2Code)
        setCode = sprintf('%s\n\n%s', set2Code, setCode);
    end
        
end