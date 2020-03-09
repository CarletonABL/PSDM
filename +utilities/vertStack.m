function B = vertStack(A, fromDimension)
    % VERTSTACK Takes a page matrix A and stacks dimension "fromDimension"
    % along the first dimension.

    if nargin < 2 || isempty(fromDimension)
        fromDimension = 3;
    end
    
    assert(fromDimension ~= 1, "Can't stack from dimension 1!");
    
    [n, m, p] = size(A);
    fromSize = size(A, fromDimension);
    
    sz = [n * fromSize, m, p];
    sz(fromDimension) = 1;
    
    B = zeros(sz);
    
    for i = 1:fromSize
        
        if fromDimension == 3
            B( (i-1)*n + (1:n), :, 1 ) = A(:, :, i);
        elseif fromDimension == 2
            B( (i-1)*n + (1:n), 1, : ) = A(:, i, :);
        end
        
    end
    
end