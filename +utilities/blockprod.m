function C = blockprod(A, B)
    % BLOCKPROD Multiplies two matrices or vectors element-wise along
    % dimension 3. Note, this can likely be done more efficiently. If
    % codegen is done it shouldn't be too bad.
    
    assert(size(A, 3) == size(B, 3) || ...
           size(B, 3) == 1 || ...
           size(A, 3) == 1, "B and A must have the same dimension 3");
       
    assert(size(A, 2) == size(B, 1), "Matrices cannot be multiplied!");
    
    N = max( size(A, 3), size(B, 3) );
    
    single_A = size(A, 3) == 1;
    single_B = size(B, 3) == 1;
    
    
    % If either A or B have no 3rd dimension, then we can get away with
    % just a normal matrix multiplication + some rearrangements
    if single_A
        B_cat = permute( utilities.vertStack(permute(B, [2 1 3])), [2 1 3]);
        C = reshape( A*B_cat, [size(A, 1), size(B, 2), N]);
        return;
    elseif single_B
        A_cat = utilities.vertStack(A);
        C = permute( reshape( permute(A_cat*B, [2 1 3]), [size(B, 2), size(A, 1), size(A, 3)]), [2 1 3]);
        return;
    else
        % Otherwise need to use a loop
        C = coder.nullcopy(zeros(size(A, 1), size(B, 2), N));
        
        % Multiply out in a loop. Use parallel computing if compiled code.
        if coder.target('matlab') || N < 20
            for i = 1:N
                C(:, :, i) = A(:, :, i) * B(:, :, i);
            end
        else
            % Else, try to use mex, catch otherwise
            c = PSDM.config;
            if coder.target('matlab') && c.use_mex
                try
                    C = utilities.blockprod_mex(A, B);
                    return;
                catch e
                    warning("utilities not compiled. You should run utilities.make!");
                    disp(e);
                end
            end
            
            parfor i = 1:N
                C(:, :, i) = A(:, :, i) * B(:, :, i);
            end
        end
        
    end
end