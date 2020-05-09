function B = rrefQR2(A, use3p_in)
    % RREFQR. A more robust routine to calculate the RREF form of A. Note,
    % for unexplained reasons, this only works when m > n.
    %
    % utilities.reffQR(A, tol, compact, use3p)
    
    if nargin < 2 || isempty(use3p_in)
        c = PSDM.config;
        use3p = c.use_iterative_refinement;
    else
        use3p = use3p_in;
    end
    
    % Do QR decomposition (don't need Q matrix)
    [~, R, Pi] = qr(A);
    
    % Calculate tolerance
    tol = min(max(size(A))*eps(class(A)), sqrt(eps(class(A)))) * abs(R(1, 1));

    ell = nnz( abs(diag(R)) > tol);
    
    R1 = R(1:ell, 1:ell);
    R2 = R(1:ell, (ell+1):end);
    %ind = 1:size(A, 2);
    %indSort = ind*Pi';
    %Pia = Pi( 1:ell, sort(indSort(1:ell)));% Pi( sort(indSort(1:ell)), : );
    Pi1a = Pi( 1:ell, :);
    Pi1 = Pi1a(:, any( abs(Pi1a) > eps, 1));
    
    
    % Solve for the last ell-p columns of B (unpermuted)
    if use3p
        Bp2 = coder.nullcopy( zeros( size(R1, 1), size(R2, 2) ) );
        Bp2 = utilities.mldivide2(R1, R2);
    else
        Bp2 = R1 \ R2;
    end
    
    B = Pi1' * [ eye(ell), Bp2 ] * Pi';
    
    %if ~utils.eqtol(B, Bt, [], true)
    %    keyboard;
    %end
    %Bt = utilities.rrefQR(A, [], true, true);
    %[Q, R, Pi] = qr(A);
    %Q1 = Q(1:ell, 1:ell);
    %Thetap1 = Q1*[R1, R2]*Pi';
    %Thetat1 = Q1*R1*Pi1;

        
end