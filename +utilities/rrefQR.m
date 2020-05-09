function Prref = rrefQR(A, tol_in, compact_in, use3p_in)
    % RREFQR. A more robust routine to calculate the RREF form of A. Note,
    % for unexplained reasons, this only works when m > n.
    %
    % utilities.reffQR(A, tol, compact, use3p)
    
    [m,n] = size(A);
    
    if nargin < 3 || isempty(compact_in)
        compact = false;
    else
        compact = compact_in;
    end
    if nargin < 4 || isempty(use3p_in)
        c = PSDM.config;
        use3p = c.use_iterative_refinement;
    else
        use3p = use3p_in;
    end

    % Do QR decomposition (don't need Q matrix)
    [~, R, p] = qr(A, 0);
    
    if nargin < 2 || isempty(tol_in)
        tol = min(max(size(A))*eps(class(A)), sqrt(eps(class(A)))) * abs(R(1, 1));
    else
        tol = tol_in;
    end
    
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
    coder.extrinsic('utilities.mldivide2');
    B1 = eye(b);
    if use3p
        B2 = coder.nullcopy( zeros( size(R1, 1), size(R2, 2) ) );
        B2 = utilities.mldivide2(R1, R2);
    else
        B2 = R1 \ R2;
    end

    % Concat and sort into P
    P = zeros(size(Rt));
    P(:, rowMask) = B1( p_trim, :);
    P(:, ~rowMask) = B2( p_trim, :);
    P(:, p) = P;

    if ~ compact && m > b
        Prref = vertcat( P, zeros( size(A, 1) - b, size(A, 2) ) );
    else
        Prref = P;
    end
        
end