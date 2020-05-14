function [vars, names, code] = makePhiCode(vars, names, code, PhiName, opt)
    % MAKEPTHETACODE Generates the code for defining PTheta in a function

    tol = 1e-11;
    
    % Get vars
    DOF = vars.DOF;
    P = vars.P;
    Theta = vars.Theta;

    % Calculate Phi_b
    Phi = zeros( size(P, 1), DOF );
    for i = 1:DOF
        Phi(:, i) = P(:, :, i) * Theta;
    end
    
    % Pre-multiply in gravity terms
    gravMask = ~any( vars.E( (1:(2*DOF)) + 3*DOF, :) > 0, 1);
    Phi(gravMask, :) = Phi(gravMask, :) .* utilities.g;
    
    % Round away small numbers
    Phi( abs(Phi) < tol ) = 0;
    Phi( abs(Phi - 1) < tol) = 1;
    
    Phi_mask = abs(Phi) > tol;
    
    % Do same thing for P
    P(gravMask, :, :) = P(gravMask, :, :) .* utilities.g;
    P( abs(P) < tol ) = 0;
    P( abs(P - 1) < tol) = 1;
    P = utils.ratRound(P, 1e-10);
    P_mask = abs(P) > tol;
    vars.P = P;
    vars.P_mask = P_mask;

    if strcmp(opt.alg, 'ID')
        % Inverse dynamics

        if strcmp(opt.combine_type, 'gradual')
            
            if ~opt.explicite_Phi
                code.Phi = sprintf('%s = coder.const(%s);', PhiName, mat2str(Phi( Phi_mask(:) ) ));
            end
        
        elseif strcmp(opt.tau_type, 'matrix')
            
            % Convert to string
            Phi_str = mat2str(Phi);
    
            % Convert to code
            code.Phi = sprintf('%s = coder.const(%s);', PhiName, Phi_str);
            
        elseif strcmp(opt.tau_type, 'vector')
            
            % Split up phi into separate columns for each joint to avoid
            % unecessary zero multiplications.
            for i = 1:DOF
                code.Phi_i{i} = sprintf('Phi_b_%d = coder.const(%s);', i, mat2str(Phi(Phi_mask(:, i), i)));
            end
            % Concatenate
            code.Phi = strjoin(code.Phi_i, '\n');
            %code.Phi = sprintf('Phi_b = cell(%d, 1);\n%s', DOF, code.Phi);
            
        end
        
        vars.Phi_mask = Phi_mask;
        
    else
        
        % Forward dynamics
            
        % First, do induced
        Phi_ind = Phi( ~vars.accelMask, : );
        Phi_ind_mask = abs(Phi_ind) > tol;

        % Split up phi into separate columns for each joint to avoid
        % unecessary zero multiplications.
        for i = 1:DOF
            code.Phi_ind_i{i} = sprintf('Phi_ind_%d = coder.const(%s);', i, mat2str(Phi_ind(Phi_ind_mask(:, i), i)));
        end
        % Concatenate
        code.Phi_ind = strjoin(code.Phi_ind_i, '\n');

        % Now, do the Phi_accels
        c = 0;
        Phi_acc_i = cell(DOF, 1);
        Phi_acc_i_mask = cell(DOF, 1);
        for i = 1:DOF

            Phi_acc_i{i} = Phi( vars.accelMask_i(i, :), : );
            Phi_acc_i_mask{i} = abs(Phi_acc_i{i}) > tol;

            for j = 1:DOF

                % We will leverage the symmetry of D so no need to
                % calculate the bottom half of the matrix
                if i > j
                    continue;
                end

                % Calculate code
                vars.Phi_acc_ij{i, j} = Phi_acc_i{i}(Phi_acc_i_mask{i}(:, j), j);
                if ~isempty(vars.Phi_acc_ij{i, j})
                    c = c+1;
                    code.Phi_acc_ij{c} = sprintf('Phi_acc_%d%d = coder.const(%s);', i, j, mat2str( vars.Phi_acc_ij{i, j} ) );
                end
            end

        end
        code.Phi_acc = strjoin(code.Phi_acc_ij, '\n');

        % Concatenate both
        code.Phi = sprintf('%s\n%s', code.Phi_ind, code.Phi_acc);
            
        
        vars.Phi_ind = Phi_ind;
        vars.Phi_ind_mask = Phi_ind_mask;
        vars.Phi_acc_i = Phi_acc_i;
        vars.Phi_acc_i_mask = Phi_acc_i_mask;
        
    end
    
    names.Phi = PhiName;
    vars.Phi = Phi;
    
end