function G = stc_gravityVector(Q, Xtool, typeID)
    % stc_gravityVector Returns gravity vector for the robot.
    %
    %   [G] = Robot.stc_gravityVector(Q, Xtool)
    %   [G] = Robot.stc_gravityVector(Q, Xtool, 1) returns only the EE effect.
    %   [G] = Robot.stc_gravityVector(Q, Xtool, 2) returns only the manipulator
    %       effects.
    %
    % G is a DOF x N vector (N being the number of poses you'd like to
    % calculate)
    %
    % Q, is DOF x N vectors (rad).
    %
    % Xtool is a a 1 x 10 vector of the tool inertial properties (m, rcx, rcy,
    %   rcz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    
    DOF = DensoAlt5.DOF;
        
    assert(size(Q, 1) == DOF && size(Q, 2) >= 1 && size(Q, 3) == 1, "Q must be a DOF x N matrix of joint angles, in radians!");
    
    if nargin < 3 || isempty(typeID)
        if nargin < 2 || isempty(Xtool)
            typeID = uint8(2);
        else
            typeID = uint8(0);
        end
    end
    
    if coder.target('matlab')
        
        try

            G = attemptGravity(Q, Xtool, typeID);

        catch

            warning("Couldn't run codegen dynamics - likely robot is not built! Recommend building robot.");

        end
        
    else
        
        G = attemptGravity(Q, Xtool, typeID);
            
    end
        
end

function G = attemptGravity(Q, Xtool, typeID)

    N = size(Q, 2);
    DOF = DensoAlt5.DOF;
    G = zeros(DOF, N);
    
    for i = 1:N

        if typeID == 0 || typeID == 2
            G(:, i) = G(:, i) + codegen_gravityVector(Q(:, i));
        end

        if typeID == 0 || typeID == 1
            G(:, i) = G(:, i) + codegen_EE_gravityVector(Q(:, i), Xtool);
        end
        
    end
    
end