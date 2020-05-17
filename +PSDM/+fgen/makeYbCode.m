function tauCode = makeYbCode( E, P, tauName, Yname, names, opt)

    % Break out some variables
    DOF = size(P, 3);
    ell = size(P, 2);
    tol = 1e-11;
    P_mask = abs(P) > tol;
    m = size(E, 2);
    n = size(E, 1);
    
    %% Check if empty set was given
    if m == 0 || n == 0
        tauCode = sprintf('%s = 0.0;\n', tauName);
        return;
    end
    
    %% Loop through joints
    
    % Initialize vector to keep track of which elements of Y were skipped
    % (are identically zero)
    wasSkipped = zeros((ell*DOF), 1, 'logical');
    tau_j = cell(ell*DOF, 1);
    
    % Figure out percentage of "zero" elements in this matrix. If above a
    % threshold, instead of doing a matrix multiplication, just mulitply
    % the elements individually.
    nels = DOF*ell;
    nz = 0;
    for k = 1:ell
        for j = 1:DOF
            if ~any(P_mask(:, k, j))
                nz = nz+1;
            end
        end
    end
    
    % Do individual mult?
    doIndiv = nz/nels > 0.35 && ~opt.return_Y;
    
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
            c2 = c2+1;
            c4 = c4 + 1;
            
            % Get reduced E matrix, and corresponding elements of gamma
            Su_t = E(colMask, rowMask);
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
            
            if ~doIndiv
                name = sprintf('%s(%d)', Yname, sub2ind([DOF, ell], j, k));
            else
                name = sprintf('%s_%d', Yname, sub2ind([DOF, ell], j, k));
            end

            % Call genCode for element
            tau_j{c2} = PSDM.fgen.genCode(Su_t, ...
                name, ...
                sprintf('%s_%d', Yname, c2),...
                names_t, opt, 1);
            
            if doIndiv
                codeRowEls{c4} = sprintf('%s.*Theta(%d)', name, k);
            end
            
        end
        
        % Code row, if doing things individually
        if doIndiv
            if c4 > 0
                codeRows{j} = strjoin(codeRowEls, '+');
            else 
                codeRows{j} = '0.0';
            end
        end
                
    end
        
    %% Generate code to combine tau
    
    if ~ doIndiv
        tauCode = sprintf('%s = zeros(%d, 1); \n %s', ...
            Yname, ell*DOF, strjoin(regexprep(tau_j(1:c2), '\n+', '\n'), '\n'));
    else
        tauCode = strjoin(regexprep(tau_j(1:c2), '\n+', '\n'), '\n');
    end
            
    % Append final multiplication
    if opt.return_Y
        tauCode = sprintf(['%s \n', ...
                            'Y(:, :, i) = reshape(%s, [%d, %d]);\n', ...
                            'if calcTau\n', ...
                            '\t%s = reshape(%s, [%d, %d]) * Theta;\n', ...
                            'end\n'], ...
                            tauCode, Yname, DOF, ell, tauName, Yname, DOF, ell);
    else
        if ~doIndiv
            tauCode = sprintf('%s \n%s = reshape(%s, [%d, %d])*Theta;', tauCode, tauName, Yname, DOF, ell);
        else
            if DOF > 1
                tauCode = sprintf('%% Indiv done. \n%s\n%s = [%s];', tauCode, tauName, strjoin(codeRows, ';\n') );
            else
                tauCode = sprintf('%% Indiv done. \n%s\n%s = %s;', tauCode, tauName, strjoin(codeRows, ';') );
            end
        end
    end


   
end