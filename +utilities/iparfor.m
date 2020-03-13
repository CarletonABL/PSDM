function res = iparfor( func, N, sz )
    % IPARFOR A small function that evaluates a function "func" either in a
    % for loop (if parallel processing not enabled) or in a parallel for
    % loop (if doing code generation, or if a parallel pool is already enabled.
    % This allows for cleaner code than multiple if statements.
    %
    % INPUTS:
    %   - func: a function handle which accepts a single argument, i, that
    %       is the loop variable, and returns the "result" for that loop.
    %   - N: The number of iterations to perform (i.e. i = 1:N)
    %   - sz: The size of the output of the function. 
    
    assert( numel(N) == 1 && N >= 1, "N must be a scalar integer, greater than 1" );
    
    if coder.target('matlab')
        doPar = ~isempty(gcp('nocreate'));
    else
        doPar = true;
    end
    
    res = zeros( [sz N] );
    
    if doPar
        parfor i = 1:N
            res_i = func(i);
            coder.varsize('res_i', [inf inf], [1 1]);
            res(:, :, i) = res_i;
        end
    else
        for i = 1:N
            res_i = func(i);
            coder.varsize('res_i', [inf inf], [1 1]);
            res(:, :, i) = res_i;
        end
    end
    
end