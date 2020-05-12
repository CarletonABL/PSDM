function Yp = generateYp(Q, Qd, Qdd, E)
    % GENERATEYP Generates the regressor matrix Yp for a list of joint
    % angles Q, Qd, Qdd and an exponent matrix E.
    %
    % Yp = generateYp(Q, Qd, Qdd, E)
    
    %% Preamble
    
    if size(E, 2) == 0
        
        % Cancel out if empty matrix
        Yp = zeros(size(E));
        return;
        
    end
    
    % Run mex, if possible
    c = PSDM.config;
    if coder.target('matlab') && size(Q, 2) < 40000 && c.use_mex_basic
         try
            Yp = PSDM.generateYp_mex(Q, Qd, Qdd, E);
            return; 
         catch
             warning("PSDM is not compiled! PSDM.generateYp will run slowly without compilation. Recommend running PSDM.make");
         end
    end
    
    %% Special functionality for symbolic gamma
    if coder.target('matlab') && isa(Q, 'sym')
        
        % Calculate the values of the generator function
        gamma = vertcat( Q, sin(Q), cos(Q), Qd, Qdd );
        
        Yp = prod(gamma .^ sym(E), 1);
        return;
        
    end
    
    %% Otherwise, do function normally
    doGravity = true;
    DOF = size(Q, 1);
    
    % Permute joint angles into 3rd dimension
    Q = permute(Q, [1 3 2]);
    Qd = permute(Qd, [1 3 2]);
    Qdd = permute(Qdd, [1 3 2]);
    
    % Calculate the values of the generator function
    gamma = vertcat( Q, sin(Q), cos(Q), Qd, Qdd );
    
    % Q, cos(Q) and Qd can be squared. Take these parts out to "optimize"
    % the computation
    Eexp = vertcat( E > 0, E( (1:DOF), : ) > 1, E( 2*DOF+(1:DOF), : ) > 1, E(3*DOF+(1:DOF), : ) > 1 );
    gamma_exp = vertcat(gamma, gamma( (1:DOF), 1, : ), gamma( 2*DOF + (1:DOF), 1, : ), gamma( 3*DOF + (1:DOF), 1, :) );

    % Indexing is the slowest operation here. Make it a bit better by
    % pre-trimming E and gamma of any variables that do not occur in any of
    % the terms
    trimMask = any(Eexp, 2);
    Eexp_trim = Eexp(trimMask, :);
    gamma_exp_trim = gamma_exp(trimMask, :, :);
    
    % Get sizes
    M = size(E, 2);
    N = size(Q, 3);
    
    Yp = permute( ...
            utilities.iparfor( ...
                @(m) permute( prod( gamma_exp_trim( Eexp_trim(:, m), 1, : ), 1 ), [3 1 2]), ...
                M, [N, 1], false), ...
            [1 3 2]);
    
    if doGravity
        gravMask = ~any( E(3*DOF + (1:(2*DOF)), :) > 0, 1);
        Yp(:, gravMask) = Yp(:, gravMask) * utils.g;
    end
    
end