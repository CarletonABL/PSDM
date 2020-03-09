function [D, C, G] = stc_EOM(Q, Qd, Xtool, typeID, desiredMatrix)
    % EOM Returns the mass, coriolis and gravity matrices for the robot.
    %
    %   [D, C, G] = Robot.stc_EOM(Q, Qd, Xtool)
    %   [D, C, G] = Robot.stc_EOM(Q, Qd, Xtool, 1) returns only the EE effect.
    %   [D, C, G] = Robot.stc_EOM(Q, Qd, Xtool, 2) returns only the manipulator
    %       effects.
    %   [D, C, G] = Robot.stc_EOM(Q, Qd, Xtool, ___, desiredMatrix) allows the
    %       user to only return one of the 3 matrices (0 = all matrices, 1
    %       = only mass matrix D, 2 = only Coriolis matric C, and 3 = only
    %       Gravity vector G.
    %
    % D, C are DOF x DOF x N matrices. G is a DOF x 1 x N vector (where N
    % is the number of poses you'd like to calculate).
    %
    % Q, Qd are DOF x N vectors (rad, and rad/s respectively).
    %
    % Xtool is a a 1 x 10 vector of the tool inertial properties (m, rcx, rcy,
    %   rcz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    
    DOF = DensoAlt5.DOF;
    
    assert(size(Q, 1) == DOF && size(Q, 2) >= 1 && size(Q, 3) == 1, "Q must be a DOF x N matrix of joint angles, in radians!");
    assert(all(size(Q) == size(Qd)), "Q and Qd must all have the same dimensions.");
    
    if nargin < 4 || isempty(typeID)
        if nargin < 3 || isempty(Xtool)
            typeID = uint8(2);
        else
            typeID = uint8(0);
        end
    end
    
    if nargin < 5 || isempty(desiredMatrix)
        desiredMatrix = uint8(0);
    end
    
    if coder.target('matlab')
        
        try

            [D, C, G] = attemptEOM(Q, Qd, Xtool, typeID, desiredMatrix);

        catch

            warning("Couldn't run codegen dynamics - likely robot is not built! Recommend building robot.");

        end
        
    else
        
        [D, C, G] = attemptEOM(Q, Qd, Xtool, typeID, desiredMatrix);
        
    end
        
end

function [D, C, G] = attemptEOM(Q, Qd, Xtool, typeID, desiredMatrix)

    N = size(Q, 2);

    DOF = size(Q, 1);
    
    doD = desiredMatrix == 0 || desiredMatrix == 1;
    doC = desiredMatrix == 0 || desiredMatrix == 2;
    doG = desiredMatrix == 0 || desiredMatrix == 3;
    
    D = zeros(DOF, DOF, N);
    C = zeros(DOF, DOF, N);
    G = zeros(DOF, 1, N);
    
    for i = 1:N

        if typeID == 0 || typeID == 2
            if doD; D(:, :, i) = D(:, :, i) + codegen_massMatrix(Q(:, i)); end
            if doC; C(:, :, i) = C(:, :, i) + codegen_coriolisMatrix(Q(:, i), Qd(:, i)); end
            if doG; G(:, :, i) = G(:, :, i) + codegen_gravityVector(Q(:, i)); end
        end

        if typeID == 0 || typeID == 1
            if doD; D(:, :, i) = D(:, :, i) + codegen_EE_massMatrix(Q(:, i), Xtool); end
            if doC; C(:, :, i) = C(:, :, i) + codegen_EE_coriolisMatrix(Q(:, i), Qd(:, i), Xtool);end
            if doG; G(:, :, i) = G(:, :, i) + codegen_EE_gravityVector(Q(:, i), Xtool); end
        end
        
    end
    
end