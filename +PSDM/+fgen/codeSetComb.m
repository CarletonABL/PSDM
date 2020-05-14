function [setNames, code] = codeSetComb(Su, Phi, name, keepAllElements, code, vars, opt, ...
                                 Su1, set1Names, set1Code, ...
                                 Su2, set2Names, set2Code)
    % Generates the actual code for a set.

    % Get size of set
    Mu = size(Su, 2);
    Mu2 = size(Su2, 2);
    
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
    elCode1 = repelem({''}, Mu);
    elCode2 = repelem({''}, Mu);
    elCode3 = repelem({''}, Mu);
    setNames = repelem({''}, Mu);
    
    DOF = vars.DOF;
    Phi = vars.Phi;
    Phi_mask = vars.Phi_mask;
    
    % Loop through each element of unique set
    c = 0;
    for i = 1:Mu2
        
        % Break out element 2 function
        s2 = Su2(:, i);
        
        % Find name for element 2
        s2_empty = all(s2 == 0);
        
        % Find matching element 1's
        s1_ind = find( all( S2 == s2, 1) );
        
        s1 = S1( :, s1_ind );
        s1_ind_min = findS1Indices(s1, Su1);
        
        % Sum and multiply all s1 elements
        c = c+1;
        
        Phi_ij = cell(DOF, 1);
        Phi_ij_mask = cell(DOF, 1);
        Phi_ij_code = cell(DOF, 1);
        Ya_ij_code = cell(DOF,1);
        Ya_i_codes = cell(DOF,1);

        % Loop through each joint
        for j = 1:DOF
            
            Phi_ij{j} = Phi(s1_ind, j);
            Phi_ij_mask{j} = abs(Phi_ij{j}) > 1e-11;
            if any(Phi_ij_mask{j}, 'all')
                Phi_ij_code{j} = sprintf('Phi_%s_%d_%d = coder.const(%s);', ...
                            name, i, j, mat2str( Phi_ij{j}( Phi_ij_mask{j} ) ) );
                Ya_ij_code{j} = sprintf('%s_a_%d_%d = [%s];', ...
                    name, i, j, strjoin(set1Names(s1_ind_min(Phi_ij_mask{j})), ','));
                Ya_i_codes{j} = sprintf('%s_a_%d_%d * Phi_%s_%d_%d', name, i, j, name, i, j);
            else
                Phi_ij_code{j} = '';
                Ya_ij_code{j} = '';
                Ya_i_codes{j} = '0.0';
            end
            
        end
                
        elCode1{c} = strjoin(Phi_ij_code, '\n');
        elCode2{c} = strjoin(Ya_ij_code, '\n');
                
        elCode3{c} = sprintf('[%s]', strjoin(Ya_i_codes, ';'));
        setNames{i} = sprintf('%s(%d)', name, i);
        
        
    end
    
    % Now, concatenate everything together.
    if c > 0
        setCode2 = strjoin(elCode2(1:c), '\n');
        setCode3 = PSDM.fgen.assignVector( sprintf('%s_a', name), elCode3(1:c), opt );
        setCode4 = PSDM.fgen.assignVector( sprintf('%s_b', name), set2Names(1:c), opt);
        setCode5 = ''; %sprintf('%s = %s_a * %s_b;\n', name, name, name);
        setCode = sprintf('%s\n\n%s\n\n%s\n\n%s', setCode2, setCode3, setCode4, setCode5);
    else
        if keepAllElements
            setCode = sprintf('%s = 1.0;', name);
        else
            setCode = '';
        end
    end
    
    if exist('set1Code', 'var') &&  ~ isempty(set1Code)
        setCode = sprintf('%s\n\n%s', set1Code, setCode);
    end
    
    if exist('set2Code', 'var') && ~ isempty(set2Code)
        setCode = sprintf('%s\n\n%s', set2Code, setCode);
    end
    
    code.S = setCode;
    code.Phi = sprintf('%s\n\n%s', strjoin(elCode1(1:c), '\n'));
        
end


function s1_ind_min = findS1Indices(s1, Su1)
    
    s1_ind_min = zeros(1, size(s1, 2));
    for i = 1:size(s1, 2)
        s1_ind_min(i) = find( all( Su1 == s1(:, i), 1), 1 );
    end

end