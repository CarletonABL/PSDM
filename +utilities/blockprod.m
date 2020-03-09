function C = blockprod(A_in, B_in)
    % BLOCKPROD Multiplies two matrices or vectors element-wise along
    % dimension 3. Note, this can likely be done more efficiently. If
    % codegen is done it shouldn't be too bad.
    
    assert(size(A_in, 3) == size(B_in, 3) || ...
           size(B_in, 3) == 1 || ...
           size(A_in, 3) == 1, "B and A must have the same dimension 3");
       
    assert(size(A_in, 2) == size(B_in, 1), "Matrices cannot be multiplied!");
        
    % Else, try to use mex, catch otherwise
    if coder.target('matlab')
        try
            C = utilities.blockprod_mex(A_in, B_in);
            return;
        catch
            warning("utilities not compiled. You should run utilities.make!");
        end
    end
    
    N = max( size(A_in, 3), size(B_in, 3) );
    
    C = zeros(size(A_in, 1), size(B_in, 2), N);
    
    if size(A_in, 3) == 1
        A = repmat(A_in, [1 1 N]);
    else
        A = A_in;
    end
    
    if size(B_in, 3) == 1
        B = repmat(B_in, [1 1 N]);
    else
        B = B_in;
    end
    
    for i = 1:N
        C(:, :, i) = A(:, :, i) * B(:, :, i);
    end
    
end