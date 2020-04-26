function [vars, names, code] = makeUpsilonCode(vars, names, code, opt)

    vars.DOF = size(vars.P, 3);
    DOF = vars.DOF;
    
    vars.Up = vars.E(1:(3*DOF), :);
   
   [vars, names, code] = PSDM.fgen.makeSetCode(vars.Up, 'Up', vars, names, code, opt);
   vars.Up1 = vars.Su;
   names.upsilon = names.s;
   code.Up = code.S;
   
end