function [Jp, condNum] = stc_jacobianPoint(r, i, Q)
    % stc_JacobianPoint  Get the transformation matrix of the robot of a point,
    % relative to one of the 6 frames.
    %
    %   J = Robot.stc_jacobianPoint(r, i, Q) gets the 6 x n jacobian matrix for the ith
    %   frame, i = {1-6}, of the pose Q, with the translation vector r
    %   (relative to the ith frame).
    %
    %   [J, condNum] = Robot.stc_jacobianPoint(__) additionally returns the
    %   condition number of the jacobian.
    
    DOF = DensoAlt5.DOF;
    
    assert(size(Q, 1) == DOF, "Q must be a DOF x N matrix");
    assert(size(r, 1) == 3, "r must have 3 rows (x y z)");
    assert(size(r, 2) == size(Q, 2) || size(r, 2) == 1, "Q and r must have the same number of columns!");
    assert(numel(i) == 1 || numel(i) == size(Q, 2), "i must be a scalar or a vector of the same size as the number of columns in Q!");

    n = size(Q, 2);
    
    Jp = zeros(6, DOF, n);
    
    variable_i = numel(i) > 1;
    variable_r = size(r, 2) > 1;
    doCond = nargout > 1;
    condNum = zeros(n, 1);

    for k = 1:n
        
        if variable_i
            i_k = i(k);
        else
            i_k = i;
        end
        
        if variable_r
            r_k = r(:, k);
        else
            r_k = r;
        end
        
        Jp(:, :, k) = codegen_jacobianPointSingle(Q(:, k), i_k, r_k);
        
        if doCond
            condNum(k) = cond(Jp(:, :, k));
        end
        
    end
    
end
