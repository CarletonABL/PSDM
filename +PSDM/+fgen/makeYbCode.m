function [tauCode, names, code] = makeYbCode( E, P, tauName, Yname, names, code, opt)

    % Break out some variables
    DOF = size(P, 3);
    ell = size(P, 2);
    tol = 1e-11;
    P_mask = abs(P) > tol;
    m = size(E, 2);
    n = size(E, 1);
    
    %% Check if empty set was given
    if m == 0 || n == 0
        tauCode = sprintf('\t\tdouble %s = 0.0;\n', tauName);
        return;
    end
    
    %% Loop through joints
    
    % Initialize vector to keep track of which elements of Y were skipped
    % (are identically zero)
    wasSkipped = zeros((ell*DOF), 1, 'logical');
    tau_j = cell(ell*DOF, 1);
    extra_j = cell(ell*DOF, 1);
    
    c1 = 0; % Count position in Y (index)
    c2 = 0; % Count position in non-skipped vector of Y
    
    % Loop through rows of Y
    for j = 1:DOF
        
        codeRowEls = cell(0,1);
        c4 = 0; % Number of elements in current row that are nonzero.
        
        % Loop through columns of Y
        for k = 1:ell       
        
            % Increment counter
            c1 = c1+1;

            % Trim out rows with no contribution to this joint
            rowMask = P_mask(:, k, j);
            colMask = any(E(:, rowMask) > 0, 2);
            
            % Assign value to skip
            if ~any(rowMask)
                wasSkipped(c1) = true;
                continue;
            end
            
            % Increment counter 
            c4 = c4 + 1;
            
            % Get reduced E matrix, and corresponding elements of gamma
            Su_t = E(colMask, rowMask);
            
            if isempty(Su_t)
                % All we need is the theta vector here.
                coef = mat2str(P(rowMask, k, j));
                switch coef
                    case '1'
                        codeRowEls{c4} = sprintf('Theta[%d]', k-1);
                    case '-1'
                        codeRowEls{c4} = sprintf('-Theta[%d]', k-1);
                    otherwise
                        codeRowEls{c4} = sprintf('%s*Theta[%d]', coef, k-1);
                end
                
            else
                c2 = c2+1;
                names_t.gamma{1} = names.gamma{1}(colMask);
                names_t.gamma{2} = names.gamma{2}(colMask);

                % Store the text of each element of P in a cell array for use.
                names_t.y = cell(nnz(rowMask),1);
                c3 = 0;
                for i = 1:m
                    if rowMask(i)
                        c3 = c3+1;
                        names_t.y{c3} = mat2str(P(i, k, j));
                    end
                end

                % Get the indexed position of this value of Y
                subInd = sub2ind([DOF, ell], j, k);

                % Make name
                name = sprintf('%s_p%d', Yname, subInd);

                % Call genCode for element
                [tau_j{c2}, names, code] = PSDM.fgen.genCode(Su_t, ...
                    name, ...
                    sprintf('%s_%d', Yname, c2),...
                    names_t, names, code, opt, 1);
            
                % Add element to row code (will be concatenated with addition
                % later
                codeRowEls{c4} = sprintf('%s*Theta[%d]', name, k-1);
                
            end
            
        end
        
        % Code row, if doing things individually
        if c4 > 0
            codeRows{j} = strrep( strjoin(codeRowEls, '+'), '+-', '-');
        else 
            codeRows{j} = '0.0';
        end
                
    end
        
    %% Generate code to combine tau
    
    if c2 > 0
        tauCode = strjoin(regexprep(tau_j(1:c2), '\n+', '\n'), '\n\t\t');
    else
        tauCode = '';
    end
    
    for i = 1:DOF
        tauCode = sprintf('%s\n\t\t%s = %s;', tauCode, tauName{i}, codeRows{i});
    end
       
end
