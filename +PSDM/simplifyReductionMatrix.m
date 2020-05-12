function P = simplifyReductionMatrix(P, tol_in)
    % SIMPLIFYREDUCTIONMATRIX Transforms a set of reduction matrices Pi to
    % a slightly simpler form, by normalizing each column by the first
    % element in it.
    %
    %   Pi = PSDM.simplifyReductionMatrix(Pi);
    
    if nargin < 2 || isempty(tol_in)
        tol = 1e-10;
    else
        tol = tol_in;
    end
    
    ell = size(P, 2);
    tol2 = 1e-6;
    
    for j = 1:ell
        Pij = P(:, j, :);
        factorInd = find( abs( Pij ) > tol2, 1);
        if ~isempty(factorInd)
            factor = Pij(factorInd(1));
            P(:, j, :) = P(:, j, :) / factor;
        end
    end
    
    % Second step, set any terms less than tolerance to zero
    P(abs(P) < tol) = 0;
    
end