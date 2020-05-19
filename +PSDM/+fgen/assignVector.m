function [codeVec, names] = assignVector( name, els, names, opt )

    N = numel(els);
    codeEls = arrayfun(@(i) sprintf('\t\t%s_p%d = %s;', name, i, els{i}), 1:N, ...
        'uniformoutput', false);
    nameEls = arrayfun(@(i) sprintf('%s_p%d', name, i), 1:N, ...
        'uniformoutput', false);
    codeVec = strjoin(codeEls, '\n');
    names = PSDM.fgen.addToNames(nameEls, names);
                
end