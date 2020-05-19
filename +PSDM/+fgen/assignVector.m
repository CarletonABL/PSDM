function [codeVec, names] = assignVector( name, els, names, opt )
    % Writes the code for all the elements in a vector, assuming sequencial
    % order, such that each element is called:
    %   
    %   name_p1, name_p2, etc...
    %
    % INPUTS:
    %   - name: The prefix of the variable.
    %   - els: A cell array of the code for each element.
    %   - names: The full names object.
    %   - opt: The full options object.
    %   
    % OUTPUTS:
    %   - codeVec: The code for the vector.
    %   - names: The updated names object.

    N = numel(els);
    
    % Map each code to a line of code.
    codeEls = arrayfun(@(i) sprintf('\t\t%s_p%d = %s;', name, i, els{i}), 1:N, ...
        'uniformoutput', false);
    % Combine terms
    codeVec = strjoin(codeEls, '\n');
    
    % Make a list of all names, add it to the global names list.
    nameEls = arrayfun(@(i) sprintf('%s_p%d', name, i), 1:N, ...
        'uniformoutput', false);
    names = PSDM.fgen.addToNames(nameEls, names);
                
end