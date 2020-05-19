function [tauCode, names, code] = makeTauCode( E, P, tauName, Yname, keepY, names, code, opt)
    % MAKETAUCODE Generates real-time code from a sub-function defined by a
    % set E and P.
    %
    % INPUTS:
    %   - tauName: A DOF-sized cell of names for each torque output.
    %   - Yname: The name to use for the intermediate Y variables.
    %   - keepY: If true, code will be generated to ensure that Y is
    %       fully defined. Otherwise, many elements will be ignored.
    %   - names: The names object
    %   - code: the code object
    %   - opt: The option object.
    %
    % OUTPUTS:
    %   - tauCode: The full code for the tau
    %   - names, code: The structures for names and code, updated.

    % Break out some variables
    DOF = size(P, 3);
    ell = size(P, 2);
    tol = 1e-11;
    P_mask = abs(P) > tol;
    m = size(E, 2);
    n = size(E, 1);
    
    %% Check if empty set was given
    if m == 0 || n == 0
        tauCode = sprintf('\t\t%s = 0.0;\n', tauName);
        return;
    end
    
    %% Loop through joints
    
    % Initialize vector to keep track of which elements of Y were skipped
    % (are identically zero)
    wasSkipped = zeros((ell*DOF), 1, 'logical');
    tau_j = cell(ell*DOF, 1);
    yCode = cell(ell*DOF, 1);
    
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
                yCode{c1} = sprintf('%s[startIndY + %d] = 0.0;', Yname, (k-1)*DOF+j-1);
                continue;
            end
            
            % Increment counter 
            c4 = c4 + 1;
            
            % Get reduced E matrix, and corresponding elements of gamma
            Su_t = E(colMask, rowMask);
            
            % Check if set is empty or not. If it is, no need to call
            % genCode.
            if isempty(Su_t)
                % All we need is the theta vector here.
                
                coef = mat2str(P(rowMask, k, j));
                yCode{c1} = sprintf('%s[startIndY + %d] = %s;', Yname, (k-1)*DOF+j-1, coef);

                switch coef
                    case '1'
                        codeRowEls{c4} = sprintf('Theta[%d]', k-1);
                    case '-1'
                        codeRowEls{c4} = sprintf('-Theta[%d]', k-1);
                    otherwise
                        codeRowEls{c4} = sprintf('%s*Theta[%d]', coef, k-1);
                end
                
            else
                % Need to do recursive codegen.
                c2 = c2+1;

                % Define subset of names.
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

                % Make name for this variable
                name = sprintf('%s_p%d', Yname, subInd);
                yCode{c1} = sprintf('%s[startIndY + %d] = %s;', Yname, (k-1)*DOF+j-1, name);

                % Call genCode to recursively make code for this element.
                [tau_j{c2}, names, code] = PSDM.fgen.genCode(Su_t, ...
                    name, ...
                    sprintf('%s_%d', Yname, c2),...
                    names_t, names, code, opt, 1);
            
                % Add element to row code (will be concatenated with addition
                % later
                codeRowEls{c4} = sprintf('%s*Theta[%d]', name, k-1);
                
            end
            
        end
        
        % Code row this DOF of the torque matrix by summing the individual
        % terms.
        if c4 > 0
            codeRows{j} = strrep( strjoin(codeRowEls, '+'), '+-', '-');
        else 
            codeRows{j} = '0.0';
        end
                
    end
        
    %% Generate code to combine tau
    % Combine the codes
    if c2 > 0
        tauCode = strjoin(regexprep(tau_j(1:c2), '\n+', '\n'), '\n\t\t');
    else
        tauCode = '';
    end
    
    % Add the row codes
    for i = 1:DOF
        tauCode = sprintf('%s\n\t\t%s = %s;', tauCode, tauName{i}, codeRows{i});
    end
    
    % If Ykeep is true, then append the code necessary for that.
    if keepY
        tauCode = sprintf('%s\n\n\t\t%s', tauCode, strjoin(yCode, '\n\t\t'));
    end
       
end
