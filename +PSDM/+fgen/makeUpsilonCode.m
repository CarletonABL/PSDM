function [vars, names, code] = makeUpsilonCode(vars, names, code)

    vars.DOF = size(vars.P, 3);
    DOF = vars.DOF;
    
    vars.Up = vars.E(1:(3*DOF), :);
    vars.M = size(vars.E, 2);
    
    % Get unique elements in Upsilon
    vars.Up1 = unique(vars.Up', 'rows')';    
    vars.M1 = size(vars.Up1, 2);
    M1 = vars.M1;
    
    %% Build up code for upsilon elements
    
    code.upsilon = repelem({''}, M1);
    names.upsilon = repelem({''}, M1);
    
    code.Upsilon = sprintf('Up = coder.nullcopy(ones(%d, 1));\n\n', M1);
    
    % Loop through each upsilon element
    for i = 1:M1
        
        % Build up a string of all the multiplitive elements of this
        % function
        str = repelem({''}, 2*DOF);
        for j = 1:size(vars.Up, 1)
            if vars.Up1(j, i) > 0
                str{j} = names.gamma{vars.Up1(j, i)}{j};
            end
        end
        
        % Populate names
        names.upsilon{i} = sprintf('Up(%d)', i);
        
        % Populate code
        if any(vars.Up1(:, i) > 0)
            code.upsilon{i} = sprintf('Up(%d) = %s;', i, strjoin(str(vars.Up1(:, i)>0), ".*"));
        else
            code.upsilon{i} = sprintf('Up(%d) = 1;', i);
        end
        
    end
    
    % Produce code by combining all strings
    code.Upsilon = strcat(code.Upsilon, strjoin(code.upsilon, '\n'));
    
end