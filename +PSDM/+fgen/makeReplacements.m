function text = makeReplacements(text, vars, names, code)

    text = strrep(text, '/*SETUP1_CODE*/', code.setup1);
    text = strrep(text, '/*SETUP2_CODE*/', removeNewlines(code.setup2));
    text = strrep(text, '/*EXTRA_CODE*/', removeNewlines(code.extra));
    text = strrep(text, '/*TAU_CODE*/', removeNewlines(code.tau));
    text = strrep(text, '/*NAME_DEF*/', ...
        sprintf('double %s;\n', strjoin(unique(names.def), ', ')));
    text = strrep(text, '_FUNCTIONNAME_', names.func);

    % Replace other terms
    for i = 1:5
        text = strrep(text, sprintf('_%dDOF_', i), mat2str(i*vars.DOF));
    end
    text = strrep(text, '_ell_', mat2str(size(vars.P, 2)));
    text = strrep(text, '_Dsize_', mat2str(vars.DOF*(vars.DOF+1)/2));
    text = strrep(text, '_DOF2_', mat2str(vars.DOF^2));
    
   
end

function text = removeNewlines(text)

    % Remove extra tabs
    text = regexprep( text, '\t{2,}', '\t\t');

    % Remove extra newlines
    text = regexprep( text, '(\n\t*){2,}', '\n\n\t\t');
    
end
