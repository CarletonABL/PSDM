function S = getModelMultiplicationCount(E, P)

    m = size(E, 2);
    ell = size(P, 2);
    DOF = size(E, 1)/5;

    % Number of multiplications in E
    Nm1 = sum( sum(E>0, 1) - 1);
    
    % Number of multiplications in multiplying with Phi
    Nm2 = m*DOF;
    
    Na2 = (m-1)*DOF;
    
    S.Nm1 = Nm1;
    S.Nm2 = Nm2;
    S.Na2 = Na2;
    S.Na = Na2;
    S.Nm = Nm1 + Nm2;
    
end