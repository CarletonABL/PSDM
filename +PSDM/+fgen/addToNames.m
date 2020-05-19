function names = addToNames(name, names)
    
    if isa(name, 'cell')
        name2 = name(:);
    else
        name2 = {name};
    end
    
    names.def = [names.def; name2];
    
end