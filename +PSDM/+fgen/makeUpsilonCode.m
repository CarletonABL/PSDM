function [vars, names, code] = makeUpsilonCode(vars, names, code, opt)

    vars.DOF = size(vars.P, 3);
    DOF = vars.DOF;

    if opt.pre_multiply
        vars.Up = vars.E;
        if strcmp(opt.tau_type, 'vector')
            opt.tau_type = 'matrix';
            warning('vector tau type not supported with pre-multiply option');
        end
        name = 'Ypi';
        keepAll = true;
    else
        vars.Up = vars.E(1:(3*DOF), :);
        name = 'Up';
        keepAll = false;
    end
    Phi = vars.Phi;
    
    [vars, names, code] = PSDM.fgen.makeSetCode(vars.Up, Phi, name, keepAll, ...
            vars, names, code, opt, 1);
    vars.Up1 = vars.Su;
    names.upsilon = names.s;
    code.Up = code.S;
   
end