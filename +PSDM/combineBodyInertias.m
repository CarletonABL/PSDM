function Xcomb = combineBodyInertias(X1, X2)
    % COMBINEBODYINERTIA Combines two sets of inertia properties, centered at the same frame, into a single body.
    %
    % X1, X2, Xcomb are vectors of inertial properties in the following order:
    % [m rcx rcy rcz Ixx Iyy Izz Ixy Ixz Iyz]

    % Find combined center of gravity.

    m1 = X1(1);
    m2 = X2(1);
    mcomb = m1 + m2;

    rc1 = X1(2:4);
    rc2 = X2(2:4);
    rc_comb = (m1*rc1 + m2*rc2)./mcomb;

    % Use parallel axis theorem to move the inertias to the new COG.
    % The delta for each object
    drc1 = rc_comb - rc1;
    drc2 = rc_comb - rc2;

    % Rename variables to make code legible
    Ixx1 = X1(5); Ixx2 = X2(5);
    Iyy1 = X1(6); Iyy2 = X2(6);
    Izz1 = X1(7); Izz2 = X2(7);
    Ixy1 = X1(8); Ixy2 = X2(8);
    Ixz1 = X1(9); Ixz2 = X2(9);
    Iyz1 = X1(10); Iyz2 = X2(10);

    % Get the new inertias.
    Ixx_comb = Ixx1 + m1*( sum(drc1([2, 3]).^2) ) + Ixx2 + m2*( sum(drc2([2, 3]).^2) );
    Iyy_comb = Iyy1 + m1*( sum(drc1([1, 3]).^2) ) + Iyy2 + m2*( sum(drc2([1, 3]).^2) );
    Izz_comb = Izz1 + m1*( sum(drc1([1, 2]).^2) ) + Izz2 + m2*( sum(drc2([1, 2]).^2) );

    Ixy_comb = Ixy1 - m1*prod(drc1([1, 2])) + Ixy2 - m2*prod(drc2([1, 2]));
    Ixz_comb = Ixz1 - m1*prod(drc1([1, 3])) + Ixz2 - m2*prod(drc2([1, 3]));
    Iyz_comb = Iyz1 - m1*prod(drc1([2, 3])) + Iyz2 - m2*prod(drc2([2, 3]));

    Xcomb = [mcomb, rc_comb, Ixx_comb, Iyy_comb, Izz_comb, Ixy_comb, Ixz_comb, Iyz_comb];