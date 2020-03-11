function Yp = genTermValues(Q, Qd, Qdd, E)
    % GENERATEYP Generates the regressor matrix Yp for a list of joint
    % angles Q, Qd, Qdd and an exponent matrix E.
    %
    % v is a verbosity flag (default: true).
        
    % Special functionality for symbolic gamma
    if coder.target('matlab') && isa(Q, 'sym')
        
        % Calculate the values of the generator function
        gamma = vertcat( Q, sin(Q), cos(Q), Qd, Qdd );
        
        Yp = prod(gamma .^ sym(E), 1);
        return;
        
    end
    
    % Run mex, if possible
    c = PSDM.config;
    if coder.target('matlab') && size(Q, 2) < 40000 && c.allow_mex
         try
            Yp = PSDM.genTermValues_mex(Q, Qd, Qdd, E);
            return; 
         catch
             warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
         end
    end
    
    % Otherwise, do function normally
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

    trimMask = any(Eexp, 2);

    Eexp_trim = Eexp(trimMask, :);
    gamma_exp_trim = gamma_exp(trimMask, :, :);
    
    % Get size
    M = size(E, 2);
    N = size(Q, 3);
    Yp = zeros(N, M);
    
    if coder.target('matlab')
        p = gcp('nocreate');
        doPar = ~isempty(p);
    else
        doPar = true;
    end
    
    if doPar
        parfor m = 1:M
            Yp(:, m) = prod( gamma_exp_trim( Eexp_trim(:, m), 1, : ), 1 );
        end
    else
        for m = 1:M
            Yp(:, m) = prod( gamma_exp_trim( Eexp_trim(:, m), 1, : ), 1 );
        end
    end
    
end