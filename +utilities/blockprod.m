function C = blockprod(A, B)
    % BLOCKPROD Multiplies two matrices or vectors element-wise along
    % dimension 3. Note, this can likely be done more efficiently. If
    % codegen is done it shouldn't be too bad.
    
    assert(size(A, 3) == size(B, 3) || ...
           size(B, 3) == 1 || ...
           size(A, 3) == 1, "B and A must have the same dimension 3");
       
    assert(size(A, 2) == size(B, 1), "Matrices cannot be multiplied!");
        
    % Else, try to use mex, catch otherwise
    if coder.target('matlab')
        try
            C = utilities.blockprod_mex(A, B);
            return;
        catch e
            warning("utilities not compiled. You should run utilities.make!");
            disp(e);
        end
    end
    
    N = max( size(A, 3), size(B, 3) );
    
    C = zeros(size(A, 1), size(B, 2), N);
    
    single_A = size(A, 3) == 1;
    single_B = size(B, 3) == 1;
    
    Ai = A(:, :, 1);
    Bi = B(:, :, 1);
    
    % Multiply out in a loop. Use parallel computing if compiled code.
    if coder.target('matlab')
        for i = 1:N

            if single_A
                C(:, :, i) = A * B(:, :, i);
            elseif single_B
                C(:, :, i) = A(:, :, i) * B;
            else
                C(:, :, i) = A(:, :, i) * B(:, :, i);
            end

        end
    else
        parfor i = 1:N

            if single_A
                C(:, :, i) = Ai * B(:, :, i);
            elseif single_B
                C(:, :, i) = A(:, :, i) * Bi;
            else
                C(:, :, i) = A(:, :, i) * B(:, :, i);
            end

        end
    end
end