function Pi = simplifyReductionMatrix(Pi)
    % SIMPLIFYREDUCTIONMATRIX Transforms a set of reduction matrices Pi to
    % a slightly simpler form, by normalizing each column by the first
    % element in it.
    %
    %   Pi = PSDM.simplifyReductionMatrix(Pi);
    %
    
    N = numel(Pi);
    ell = size(Pi, 2);
    tol = 1e-6;
    
    for j = 1:ell
        Pij = Pi(:, j, :);
        factorInd = find( abs( Pij ) > tol, 1);
        if ~isempty(factorInd)
            factor = Pij(factorInd(1));
            Pi(:, j, :) = Pi(:, j, :) / factor;
        end
    end
    