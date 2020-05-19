function name_i = findEquivNames(code_i, names, code)

    N = numel(code.all);

    % Step 1: Just search directly
    alreadyFound1 = strcmp(code_i, code.all);
    if any(alreadyFound1)
        ind = find(alreadyFound1, 1);
        name_i = names.all{ind};
        return;
    end
    
    if ~contains(code_i, '+')
        % Step 2: Search for negatives (assuming this was positive, previous negative)
        alreadyFound2 = strcmp(strcat('-',code_i), code.all);
        if any(alreadyFound2)
            ind = find(alreadyFound2, 1);
            name_i = strcat('-',names.all{ind});
            return;
        end

        % Step 3: Search for negatives (assuming this was positive, previous was negative)
        if code_i(1)=='-'
            alreadyFound3 = strcmp(strcat(code_i(2:end)), code.all);
            if any(alreadyFound3)
                ind = find(alreadyFound3, 1);
                name_i = strcat('-',names.all{ind});
                return;
            end
        end
    end
    
    % Otherwise, return null
    
    name_i = '';
    
    
    
end