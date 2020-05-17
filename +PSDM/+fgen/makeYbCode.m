function [vars, names, code] = makeYbCode(vars, names, code, opt)

    vars.DOF = size(vars.P, 3);

    % Break out some variables
    E = vars.E;
    vars.Up = E;
    P = vars.P;
    P_mask = vars.P_mask;
    m = size(E, 2);
    ell = size(P, 2);
    DOF = vars.DOF;
    
    %% Loop through joints
    
    % Initialize vector to keep track of which elements of Y were skipped
    % (are identically zero)
    wasSkipped = zeros((ell*DOF), 1, 'logical');
    
    c1 = 0; % Count position in Y (index)
    c2 = 0; % Count position in non-skipped vector of Y
    % Loop through columns of Y
    for k = 1:ell       
        
        % Loop through rows of Y
        for j = 1:DOF
            
            % Increment counter
            c1 = c1+1;

            % Trim out rows with no contribution to this joint
            colMask = any(E > 0, 2);
            rowMask = P_mask(:, k, j);
            
            % Assign value to skip
            if ~any(rowMask)
                wasSkipped(c1) = true;
                continue;
            end
            
            % Increment counter 
            c2 = c2+1;
            
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

            % Call genCode for element
            code.tau_j{c2} = PSDM.fgen.genCode(Su_t, sprintf('Y_%d', c2), names_t, opt, 1);

            % Assign value to correct index of Y
            code.tau_j{c2} = sprintf('%s\n\nY_i(%d) = Y_%d;\n', code.tau_j{c2}, sub2ind([DOF, ell], j, k), c2);
            
        end
        
    end
        
    %% Generate code to combine tau
    
    code.tau = sprintf('Y_i = zeros(%d, 1); \n %s', ...
        ell*DOF, strjoin(regexprep(code.tau_j, '\n+', '\n'), '\n'));
            
    % Append final multiplication
    if opt.return_Y
        code.tau = sprintf(['%s \n', ...
                            'Y(:, :, i) = reshape(Y_i, [%d, %d]);\n', ...
                            'if calcTau\n', ...
                            '\ttau(:, i) = reshape(Y_i, [%d, %d]) * Theta;\n', ...
                            'end\n'], ...
                            code.tau, DOF, ell, DOF, ell);
    else
        code.tau = sprintf('%s \ntau(:, i) = reshape(Y_i, [%d, %d])*Theta;', code.tau, DOF, ell);
    end


   
end