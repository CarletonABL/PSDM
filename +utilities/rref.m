function A_out = rref(A, tol_in, compact_in)
%RREF   Reduced row echelon form.
%   R = RREF(A) produces the reduced row echelon form of A.
%
%   [R,jb] = RREF(A) also returns a vector, jb, so that:
%       r = length(jb) is this algorithm's idea of the rank of A,
%       x(jb) are the bound variables in a linear system, Ax = b,
%       A(:,jb) is a basis for the range of A,
%       R(1:r,jb) is the r-by-r identity matrix.
%
%   [R,jb] = RREF(A,TOL) uses the given tolerance in the rank tests.
%
%   Roundoff errors may cause this algorithm to compute a different
%   value for the rank than RANK, ORTH and NULL.
%
%   Class support for input A:
%      float: double, single
%
%   See also RANK, ORTH, NULL, QR, SVD.

%   Copyright 1984-2017 The MathWorks, Inc. 

[m,n] = size(A);

% Compute the default tolerance if none was provided.
if (nargin < 2) || isempty(tol_in)
    tol = max(m,n)*eps(class(A))*norm(A,inf);
else
    tol = tol_in;
end

if nargin < 3 || isempty(compact_in)
    compact = false;
else
    compact = compact_in;
end

if coder.target('matlab')
    try
        A_out = utilities.rref_mex(A, tol, compact);
        return; 
    catch
        warning("utilities is not compiled! This code will run very slowly without compilation. Recommend running utilities.make");
    end
end

% Loop over the entire matrix.
i = 1; % Row index
j = 1; % Column index
while i <= m && j <= n
    
   % Find value and index of largest element in the remainder of column j.
   [p, k] = max(abs(A(i:m,j)));
   k = k+i-1;
   
   if p <= tol
       
      % The column is negligible, zero it out.
      A(i:m,j) = 0;
      j = j + 1;
      
   else

      % Swap i-th and k-th rows.
      A([i k],j:n) = A([k i],j:n);
      
      % Divide the pivot row by the pivot element.
      A(i,j:n) = A(i,j:n)./A(i,j);
      
      % Subtract multiples of the pivot row from all the other rows.
      for k = 1:m % [1:i-1 i+1:m]
          if uint32(k) ~= uint32(i)
              A(k,j:n) = A(k,j:n) - A(k,j).*A(i,j:n);
          end
      end
      
      i = i + 1;
      j = j + 1;
   end
   
end

% Trim out the bottom of the array, if desired.
if compact
    r = rank(A);
    A_out = A(1:r, :);
else
    A_out = A;
end
