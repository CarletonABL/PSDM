function names = addToNames(name, names)
    % ADDTONAMES Appends a given name or set of names to the list of all
    % names – this is used to initialize the variables in the C function.
    
    if isa(name, 'cell')
        name2 = name(:);
    else
        name2 = {name};
    end
    
    names.def = [names.def; name2];
    
end