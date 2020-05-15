function C = blockprod(A, B)
    % BLOCKPROD Multiplies two matrices or vectors element-wise along
    % dimension 3.
    %
    % C = blockprod(A, B)
    
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
        
        % First, try top run a mexed version
        % Only do this if the number of iterations is large, and 
        % The number of elements in the matrices is not crazy. Otherwise,
        % its better to use matlab's super optimized version of mtimes
        c = PSDM.config();
        if N > 20 && coder.target('matlab') && c.use_mex && numel(A(:, :, 1))+numel(B(:, :, 1))<1e6
            try
                C = utilities.blockprod_mex(A, B);
                return;
            catch e
                warning("utilities not compiled. You should run utilities.make!");
                disp(e);
            end
        end
        
        % Otherwise, execute normally
        C = coder.nullcopy(zeros(size(A, 1), size(B, 2), N));
        for i = 1:N
            C(:, :, i) = A(:, :, i) * B(:, :, i);
        end
        
    end
end