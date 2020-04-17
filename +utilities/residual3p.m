function r = residual3p(A,x,b)
% RESIDUAL3p Triple precision residual, A*x - b.
% r = residual3p(A,x,b) for matrix A, vectors x and b. 
%
% Adapted from  https://blogs.mathworks.com/cleve/2015/03/02/triple-precision-accumlated-inner-product/
% made to work with matrix b and use parallel computing for speed.

    %% Parse inputs
    assert(size(A, 1) == size(b, 1), "A and b do not match in size");
    assert(size(A, 2) == size(x, 1), "A and x do not match in size");
    assert(size(x, 2) == size(b, 2), "x and b do not match in size");

    %% Try to run mex
    if coder.target('matlab')
        try
            r = utils.residual3p_mex(A, x, b);
            return;
        catch e
            disp(e);
        end
    end

   %% Actual code
   m = size(x, 2);
   N = size(A, 1);
   
   r = permute( ...
            utils.iparfor( @(i) residual1D( A(i, :), x, b(i, :) ), ...
                           N, [1, m], false), ...
            [3, 2, 1]);
        
end

function r_i = residual1D(A_i, x, b_i)

    m = size(x, 2);
    r_i = zeros(1, m);

    for j = 1:m
        r_i(j) = utils.dot3p(A_i, x(:, j), -b_i(j));
    end

end