function Ycode = makeYMatrixCode(Y, Yname, Up1, Up1names, A1, A1names)
    % MAKEYMATRIXCODE Generates the code for defining Y in a function
    
    DOF = size(Y, 1) / 5;
    
    Up = Y(1:(3*DOF), :);
    A = Y((1:(2*DOF))+3*DOF, :);
    M = size(Up, 2);

    Yels = repelem({''}, M);
    for i = 1:M
        
        Up_ind = find( all( Up1 == Up(:, i) ), 1);
        A_ind = find( all( A1 == A(:, i) ), 1);
        
        Yels{i} = sprintf('%s.*%s', A1names{A_ind}, Up1names{Up_ind});
    end
    Ycode = sprintf('%s = [%s];', Yname, strjoin(Yels, ','));
    
end