function [vars, names, code] = makeSetCode(S, name, vars, names, code, opt)
    % MakeSetCode
    % Makes the code for a set of multiplicative functions by dividing up
    % the multiplications like a tree, recursively.
    
    % Parse inputs
    if nargin < 5 || isempty(code)
        code = struct;
    end
    
    %% Step 1: Split function
    n = size(S, 1);
    Su = unique(S', 'rows')'; % Unique set
    
    if n > 2
        % Split the set until only 2 or 1 are left
        % Best place to split is at the power of 2 which is closest but
        % lower than n.
        split = 2^floor(log(n-1)/log(2));
        
        % Divide set
        Su1 = Su(1:split, :);
        Su2 = Su((split+1):end, :);
        
        vars.Su1 = Su1;
        vars.Su2 = Su2;
        
        % Give new sets new names
        name1 = strcat(name, '_1');
        name2 = strcat(name, '_2');
        
        % Divide up names variable
        names1.gamma{1} = names.gamma{1}(1:split);
        names1.gamma{2} = names.gamma{2}(1:split);
        names2.gamma{1} = names.gamma{1}((split+1):end);
        names2.gamma{2} = names.gamma{2}((split+1):end);
        
        % Get code for these sets
        [vars1, names1, code1] = PSDM.fgen.makeSetCode(Su1, name1, vars, names1, code, opt);
        [vars2, names2, code2] = PSDM.fgen.makeSetCode(Su2, name2, vars, names2, code, opt);
        
        [vars, names, code] = PSDM.fgen.codeSet(Su, name, ...
                                                vars, names, code, ...
                                                vars1, names1, code1, ...
                                                vars2, names2, code2, opt);
        
    else
        % n is 1 or 2, so now just multiply out sets
        [vars, names, code] = PSDM.fgen.codeSetSingle(Su, name, vars, names, code, opt);
    end
    
    % Append relevant variables to variables structure
    vars.Su = Su;
    
    
end

