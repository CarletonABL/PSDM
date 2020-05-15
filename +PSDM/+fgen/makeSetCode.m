function [vars, names, code] = makeSetCode(S, Phi, name, keepAllElements, vars, names, code, opt, depth)
    % MakeSetCode
    % Makes the code for a set of multiplicative functions by dividing up
    % the multiplications like a tree, recursively.
    
    % Parse inputs
    if nargin < 5 || isempty(code)
        code = struct;
    end
    
    %% Step 1: Split function
    n = size(S, 1);

    if ~keepAllElements
        Su = unique(S', 'rows', 'stable')'; % Unique set
    else
        Su = S;
    end
    
    % Trim out rows with no contribution
    colMask = any(Su > 0, 2);
    Su_t = Su(colMask, :);
    names_t.gamma{1} = names.gamma{1}(colMask);
    names_t.gamma{2} = names.gamma{2}(colMask);
    
    if n > 2
        % Get Split point
        split = getSplit(Su_t, depth, vars.DOF);
        
        % Divide set
        Su1 = Su_t(1:split, :);
        Su2 = Su_t((split+1):end, :);      
        
        vars.Su1 = Su1;
        vars.Su2 = Su2;
        
        Phi1 = Phi;
        Phi2 = Phi;
        
        % Give new sets new names
        name1 = strcat(name, '_1');
        name2 = strcat(name, '_2');
        
        % Divide up names variable
        names1.gamma{1} = names_t.gamma{1}(1:split);
        names1.gamma{2} = names_t.gamma{2}(1:split);
        names2.gamma{1} = names_t.gamma{1}((split+1):end);
        names2.gamma{2} = names_t.gamma{2}((split+1):end);
        
        % Get code for these sets
        [vars1, names1, code1] = PSDM.fgen.makeSetCode(Su1, Phi1, name1, false, vars, names1, code, opt, depth+1);
        [vars2, names2, code2] = PSDM.fgen.makeSetCode(Su2, Phi2, name2, false, vars, names2, code, opt, depth+1);
        
        if depth > 1 || ~opt.pre_multiply
            [names.s, code.S] = PSDM.fgen.codeSet(Su_t, name, keepAllElements, opt, ...
                                                  vars1.Su, names1.s, code1.S, ...
                                                  vars2.Su, names2.s, code2.S);
        else
            [names.s, code] = PSDM.fgen.codeSetComb(Su_t, Phi, name, keepAllElements, code, vars, opt, ...
                                                      vars1.Su, names1.s, code1.S, ...
                                                      vars2.Su, names2.s, code2.S);
        end
        
    else
        % n is 1 or 2, so now just multiply out sets
        [vars, names, code] = PSDM.fgen.codeSetSingle(Su_t, name, false, vars, names, code, opt);
    end
    
    % Append relevant variables to variables structure
    vars.Su = Su;
    
    
end


function s = getSplit(Su, depth, DOF)
    % Split the set until only 2 or 1 are left
    % Best place to split is at the power of 2 which is closest but
    % lower than n.
    n = size(Su, 1);
    
    if depth > 1
        s = 2^floor(log(n-1)/log(2));
        return;
    end
    

    % Split at point which results in two sets of approximately the
    % same size. Unfortunately, need to do this in a loop
    c = zeros(n, 2);
    for i = 1:n
        c(i, 1) = size(unique(Su(1:i, :)', 'rows')', 2);
        c(i, 2) = size(unique(Su(i+1:end, :)', 'rows')', 2);
    end
    
    d = abs(c(:, 1) - DOF*c(:, 2));
    [~, s] = min(d);
    
end