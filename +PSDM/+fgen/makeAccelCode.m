function [vars, names, code] = makeAccelCode(vars, names, code, opt)
    
    DOF = vars.DOF;
    
    if strcmp(opt.alg, 'ID')
        
        if opt.pre_multiply
            code.A = '';
            return;
        end
        
        % Inverse dynamics
        Aind = (1:(2*DOF))+3*DOF;
    else
        % Forward dynamic, ignore qdd terms
        Aind = (1:DOF) + 3*DOF;
    end
    
    vars.A = vars.E(Aind, :);
    vars.A1 = unique(vars.A', 'rows')';
    vars.Ma = size(vars.A1, 2);
    
    names_a = names;
    names_a.gamma{1} = names_a.gamma{1}(Aind);
    names_a.gamma{2} = names_a.gamma{2}(Aind);
    
    [vars, names, code] = PSDM.fgen.codeSetSingle(vars.A1, 'A', false, vars, names_a, code, opt);
    
    names.A = 'A';
    names.a = names.s;
    code.A = code.S;
    
end