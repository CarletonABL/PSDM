function C = initCell(n, m, className)

    coder.inline('always')

    assert( n < 1000 );
    assert( m < 1000 );
    
    C = cell( n, m );
    for i = 1:m
        for j = 1:n
            C{n, m} = zeros(0, 0, className);
        end
    end
    
end