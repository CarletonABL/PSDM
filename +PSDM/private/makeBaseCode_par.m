function [Up1, A1, setupCode, Up1names, Up1code, A1names, A1code] = makeBaseCode(E, P)
    % Make the base code to generate the elements of A and Upsilon, which
    % is used in both the makeForwardDynamics and makeInverseDynamics
    % functions.
    
    DOF = size(P, 3);
    
    %% Make a list of upsilon vectors from input
    % Need to fix this for the updated upsilon format
    lcode1 = sprintf('g%d_1 = gi(:, %d);\n', repelem(1:DOF, 2));
    lcode2 = sprintf('g%d_2 = gi(:, %d).^2;\n', repelem(1:DOF, 2));
    scode1 = sprintf('g%d_1 = gi(:, %d);\n', repelem(1:DOF, 2)+DOF);
    ccode1 = sprintf('g%d_1 = gi(:, %d);\n', repelem(1:DOF, 2)+2*DOF);
    ccode2 = sprintf('g%d_2 = gi(:, %d).^2;\n', repelem(1:DOF, 2)+2*DOF);
    acode1 = sprintf('a%d_1 = ai(:, %d);\n', repelem(1:(2*DOF), 2));
    acode2 = sprintf('a%d_2 = ai(:, %d).^2;\n', repelem((1:DOF), 2));
    
    Zvarnames{1} = split(sprintf('g%d_1,', (1:(3*DOF))), ",");
    Zvarnames{2} = split(sprintf('g%d_2,', (1:(3*DOF))), ",");
    Avarnames{1} = split(sprintf('a%d_1,', (1:(2*DOF))), ",");
    Avarnames{2} = split(sprintf('a%d_2,', (1:(2*DOF))), ",");
    
    Up = E(1:(3*DOF), :);
    Up1 = unique(Up', 'rows')';
    A = E((1:(2*DOF))+3*DOF, :);
    A1 = unique(A', 'rows')';
    
    M = size(Up, 2);
    M1 = size(Up1, 2);
    Ma = size(A1, 2);
    
    setupCode = sprintf('%s\n\n%s\n\n%s\n\n%s\n\n%s\n\n%s\n\n%s\n\n', lcode1, lcode2, scode1, ccode1, ccode2, acode1, acode2);
    
    %% Build up code for upsilon elements
    Up1names = repelem({''}, M1);
    
    Ncores = 4;
    Nits = Ncores * 2;
    Ni = ceil(M1/Nits);
    
    Up1code = sprintf("Up_mult = coder.nullcopy(zeros(%d, %d));\n", Ni, Nits);
    Up1code = strcat( Up1code, sprintf("parfor (j = 1:%d, %d)\n", Nits, Ncores) );
    Up1code = strcat( Up1code, sprintf("\tUpi = coder.nullcopy(zeros(%d, 1));\n", Ni) );
    Up1code = strcat( Up1code, sprintf("\tswitch j\n") );

    for w = 1:Nits
        
        Up1code = strcat( Up1code, sprintf("\n\tcase %d\n", w) );
        
        Up1codes = repelem({''}, Ni);
        
        for k = 1:Ni
            
            i = k + (w-1)*Ni;
            
            if i > M1
                break;
            end

            str = repelem({''}, 2*DOF);
            for j = 1:size(Up, 1)
                if Up1(j, i) > 0
                    str{j} = Zvarnames{Up1(j, i)}{j};
                end
            end

            Up1names{i} = sprintf('Up(%d)', i);
            if any(Up1(:, i) > 0)
                Up1codes{k} = sprintf('Upi(%d) = %s;', k, strjoin(str(Up1(:, i)>0), ".*"));
            else
                % Up1codes{k} = sprintf('Upi(%d) = 1;', k);
            end
        end
        
        Up1code = strcat( Up1code, sprintf("\t\t"), strjoin(Up1codes, '\n\t\t'));
        
    end
    
    Up1code = strcat( Up1code, sprintf("\n\tend\n\tUp_mult(:, j) = Upi;\nend\n") );
    Up1code = strcat( Up1code, sprintf("Up = Up_mult(:);") );
    
    %% Build up code for acceleration elements
    A1codes = repelem({''}, Ma);
    A1names = repelem({''}, Ma);
    for i = 1:Ma
        str = repelem({''}, 2*DOF);
        for j = 1:(2*DOF)
            if A1(j, i) > 0
                str{j} = Avarnames{A1(j, i)}{j};
            end
        end
        
        A1names{i} = sprintf('A%d', i);
        if any(A1(:, i) > 0)
            A1codes{i} = sprintf('A%d = %s;', i, strjoin(str(A1(:, i)>0), ".*"));
        else
            A1codes{i} = sprintf('A%d = 1;', i);
        end
    end
    A1code = strjoin(A1codes, '\n');
    
end