function B = rrefQR(A)
    % RREFQR. A more robust routine to calculate the RREF form of A. Note,
    % for unexplained reasons, this only works when m > n.
    %
    % utilities.reffQR(A)
    
    [m,n] = size(A);

    % Do QR decomposition (don't need Q matrix)
    [~, R, p] = qr(A, 0);
    
    tol = min(max(size(A))*eps(class(A)), sqrt(eps(class(A)))) * abs(R(1, 1));

    % Make masks and sorting indices
    colMask = abs( diag(R) ) > tol;
    rowMask = horzcat( colMask', zeros(1, max(0, n-m), 'logical') );

    c = (1:numel(p))';
    [ ~, p_trim ] = sort( c(p(colMask)) );

    % Get rank
    b = nnz(colMask);

    % Form R1, R2 matrices
    Rt = R(colMask, :);
    R1 = Rt(:, colMask);
    R2 = Rt(:, ~rowMask);

    % Form P1, P2 matrices
    B1 = eye(b);
    B2 = utilities.linsolve(R1, R2);

    % Concat and sort into P
    B = coder.nullcopy( zeros(size(Rt)) );
    B(:, rowMask) = B1( p_trim, :);
    B(:, ~rowMask) = B2( p_trim, :);
    B(:, p) = B;
        
end

% Note, the above formula doesn't use the same equations as in the paper.
% This is because are using an index list vector p, rather than a
% permutation matrix Pi. Result is the same but the above is more
% efficient. However, the code below accomplishes the same result and uses
% the same equations as in the paper.

%{

function B = rrefQR2(A)
    % RREFQR. A more robust routine to calculate the RREF form of A. Note,
    % for unexplained reasons, this only works when m > n.
    %
    % utilities.reffQR2(A)
    
    % Do QR decomposition (don't need Q matrix)
    [~, R, Pi] = qr(A);
    
    % Calculate tolerance
    tol = min(max(size(A))*eps(class(A)), sqrt(eps(class(A)))) * abs(R(1, 1));

    % Get rank
    ell = nnz( abs(diag(R)) > tol);
    
    % Solve for matrices
    R1 = R(1:ell, 1:ell);
    R2 = R(1:ell, (ell+1):end);
    
    % Form Pi1
    Pi1a = Pi( 1:ell, :);
    Pi1 = Pi1a(:, any( abs(Pi1a) > eps, 1));
    
    % Solve for the last ell-p columns of B (unpermuted)
    Bp2 = utilities.linsolve(R1, R2);

    % Get reduction matrix
    B = Pi1' * [ eye(ell), Bp2 ] * Pi';
        
end

%}