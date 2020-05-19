function tag = getTag(code, opt)

    switch opt.language
        
        case 'matlab'
            tag = sprintf('%%%s%%', code);
            
        case {'C', 'c'}
            tag = sprintf('/*%s*/', code);
            
        otherwise
            error("Invalid language");
            
    end
    
end