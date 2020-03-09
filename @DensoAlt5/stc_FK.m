function T = stc_FK(Q, frameID, Ttool)
    % STC_FK(Qin, frameID, Ttool) Get the transformation matrix of
    % the robot at frame frameID with joint variables Q (n x 1)
    %
    %   T = Robot.STC_FK(Q) gets the transformation matrix for the
    %   tool frame of the pose Q.
    %
    %   T = Robot.STC_FK(Q, i), i = {-1, 1-DOF+1} returns the transform for the ith
    %   frame. A value of i = DOF+1 will return the tool frame. A value of
    %   i = -1 will return all frames.
    %
    %   T = Robot.STC_FK(Q, i, Ttool) overrides the current tool transform
    %   Ttool.
    %
    % Q [DOF x 1]: Vector of joint angles (radian)
    % Ttool [4 x 4]: Transformation matrix from the last frame to the tool
    %   frame.
    % T [4 x 4]: Page array of the corresponding transformation
    %   matrix.

    DOF = int32(DensoAlt5.DOF);
    
    assert(size(Q, 1) == DOF, "Q must be a DOF x N vector");
    
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
            error("You must specify the tool transform if you want the FK of the tool frame!");
        end
    end
    
    % Get number of joint angles to be processed
    n = size(Q, 2);
    
    % Need to run slightly different code depending on which frame
    % is returned
    if(i > 0)
        T = zeros(4, 4, n);
        for j = 1:n
            T(:, :, j) = FKSingle(Q(:, j), i, Ttool);
        end
    else
        T = zeros(4, 4, n, DOF+1);
        for j = 1:n
            T(:, :, j, :) = codegen_FK_all(Q(:, j), Ttool(1:3, :));
        end
    end
    
end


function T = FKSingle(Q, i, Ttool)

    DOF = size(Q, 1);

    if i == DOF + 1
        T = codegen_FK_tool(Q, Ttool(1:3, :));
    else
        T = codegen_FKSingle(Q, i);
    end
end

