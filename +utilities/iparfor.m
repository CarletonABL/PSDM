function res = iparfor( func, N, sz, specifySize )
    % IPARFOR A small function that evaluates a function "func" either in a
    % for loop (if parallel processing not enabled) or in a parallel for
    % loop (if doing code generation, or if a parallel pool is already enabled.
    % This allows for cleaner code than multiple if statements.
    %
    % res = iparfor( func, N, sz )
    %
    % INPUTS:
    %   - func: a function handle which accepts a single argument, i, that
    %       is the loop variable, and returns the "result" for that loop.
    %   - N: The number of iterations to perform (i.e. i = 1:N)
    %   - sz: The size of the output of the function.
    %
    % Example: Solve multiple regression problems in a loop.
    %   
    %   Yi = rand(100, 100, 500);
    %   tau = rand(100, 1, 500);
    %   
    %   theta = utils.iparfor( ...
    %           @(i) Yi(:, :, i) \ tau(:, :, i), ...
    %           500, ...
    %           [100, 1]);
    
    if nargin < 4 || isempty(specifySize)
        specifySize = true;
    end
    
    assert( numel(N) == 1 && N >= 1, "N must be a scalar integer, greater than 1" );
        
    res = zeros( [sz N] );
    res_i = coder.nullcopy(zeros( sz ));
    
    if utils.autoPar
        parfor i = 1:N
            res_i = feval(func, i);
            if specifySize
                coder.varsize('res_i', sz, [1 1]);
            else
            	coder.varsize('res_i', [inf inf], [1 1]);
            end
            res(:, :, i) = res_i;
        end
    else
        for i = 1:N
            res_i = feval(func, i);
            coder.varsize('res_i', [inf inf], [1 1]);
            res(:, :, i) = res_i;
        end
    end
    
end