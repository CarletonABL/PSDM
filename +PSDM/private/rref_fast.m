function Tmin = rref_fast(T, tol)

    if nargin < 2
        tol = 1e-14;
    end
    
    % This code isn't working properly, just do rref
    if true
        
        Tmin = utils.rref(T);
        Tmin = round(Tmin, 15);
        Tmin = Tmin(1:rank(Tmin), :);

        return;
        
    end
    
    %% Old code
    % Works for the most part, but sometimes bugs ?

    [m, n] = size(T); % N is number of parameters, m is number of samples
    
    % Find out which columns to keep
    [~, R] = qr(T);
    b = rank(R);
    
    [~, maxInd] = maxk(abs(diag(R)), b);
    
    maskKeep = zeros(n, 1, 'logical');
    maskKeep(maxInd) = true;
    
    % Initialize the matrix
    Tmin = zeros(b, n);
        
    % Solve for remaining rows
    Tmin = T(:, maxInd) \ T(: , :);
    
    Tmin = round(Tmin, round(-log10(tol)));
    
end