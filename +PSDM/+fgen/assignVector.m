function codeVec = assignVector( name, els, opt )

    % Single elements don't need to be vectorized.
    if numel(els) == 1 && strcmp(opt.language, 'matlab')
        codeVec = sprintf('%s = %s;', name, els{1});
        return;
    end

    switch opt.assign_type
        case {'vector'}
            codeVec = sprintf('%s = [%s];', name, strjoin(els, ', '));
            
        case {'newline'}
            N = numel(els);
            codeEls = repelem({''}, N);
            
            switch opt.language
                case 'matlab'
                    for i = 1:N
                        codeEls{i} = sprintf('%s(%d) = %s;', name, i, els{i});
                    end
                    codeVec = sprintf(...
                        ['%s = coder.nullcopy(zeros(1, %d));\n', ...
                         '%s'], ...
                        name, N, strjoin(codeEls, '\n'));
                case 'c'
                    for i = 1:N
                        codeEls{i} = sprintf('\t\tdouble %s_p%d = %s;', name, i, els{i});
                    end
                    codeVec = strjoin(codeEls, '\n');
            end
            
    end
    
end

%{
function codeVec = assignVector( name, els, opt )

    % Single elements don't need to be vectorized.
    if numel(els) == 1
        codeVec = sprintf('%s = %s;', name, els{1});

    else
        codeVec = sprintf('%s = [%s];', name, strjoin(els, ', '));
    end

end
%}