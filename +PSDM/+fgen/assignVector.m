function codeVec = assignVector( name, els, opt )

    % Single elements don't need to be vectorized.
    if numel(els) == 1
        codeVec = sprintf('%s = %s;', name, els{1});

    else
        codeVec = sprintf('%s = [%s];', name, strjoin(els, ', '));
    end

end