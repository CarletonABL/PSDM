function B = stack(A, stackDim, removeDim)

    error("Not complete");

    sz = size(A);
    numDim = numel(sz);
    
    isRemoveDim = (1:numDim) == removeDim;
    isStackDim = (1:numDim) == stackDim;

    sz2 = sz(~isRemoveDim);
    sz2(stackDim) = sz(stackDim) * sz(removeDim);
    B = zeros(sz2);
    
    for i = 1:sz(removeDim)
        
    end

end