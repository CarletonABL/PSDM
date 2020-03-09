function [J, condNum] = stc_jacobian(Q, frameID, Ttool)
    % STC_JACOBIAN  Get the jacobian matrix of the robot
    %
    %   J = Robot.stc_jacobian(Q) gets the 6 x n jacobian matrix for the
    %   tool frame of the pose Q. If Q is a matrix, returns a page array of
    %   each solution.
    %
    %   J = Robot.stc_jacobian(Q, i), i = {-1, 1 - DOF+1} returns the transform for the ith
    %   frame. A value of i = -1 will return all frames at once. A value of
    %   i = 7 will return the tool frame.
    %
    %   J = Robot.stc_jacobian(Q, i, Ttool) overrides the current tool
    %       transform with Ttool.
    
    DOF = DensoAltNoOpt.DOF;
    assert(size(Q, 1) == DOF && size(Q, 3) == 1, "Qin must be a DOF x N matrix of joint angles");
     
    % Parse incomplete arguments
    if nargin > 1
        i = int32(frameID);
    else
        i = int32(DOF+1);
    end
    if i ~= -1 && (i < 1 || i > DOF + 1)
        error("Invalid frameID given.");
    end
    
    % Default Ttool
    if nargin < 3 || isempty(Ttool)
        Ttool = eye(4);
        
        if i == DOF+1
            error("You must specify the tool transform if you want the jacobian of the tool frame!");
        end
    end
    
    doCond = nargout > 1;
    
    % Get number of joint angles to be processed
    n = size(Q, 2);
    
    % Need to run slightly different code depending on which frame
    % is returned
    if(i > 0)
        J = zeros(6,DOF,n);
        for j = 1:n
            Jtemp = jacobianSingle(Q(:, j), i, Ttool);
            J(:, :, j) = Jtemp;
        end
    else
        J = zeros(6, DOF, n, DOF+1);
        for j = 1:n
            J(:, :, j, :) = codegen_jacobian_all(Q(:, j), Ttool(1:3, 4));
        end
    end
    
    % Get condition number, if needed
    if doCond
        condNum = zeros(n, size(J, 4));
        for j = 1:size(J, 4)
            for k = 1:n
                condNum(k, j) = cond(J(:, :, k, j));
            end
        end
    else
        condNum = NaN(n, 1);
    end
    
end


function J = jacobianSingle(Q, i, Ttool)
    
    DOF = DensoAltNoOpt.DOF;
    
    % Call appropriate function
    if i == DOF + 1
        J = codegen_jacobian_tool(Q, Ttool(1:3, 4));
    else
        J = codegen_jacobianSingle(Q, i);
    end
    
end
