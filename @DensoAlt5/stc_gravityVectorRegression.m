function [tau, Y] = stc_gravityVectorRegression(Q, Theta, Xtool)
    % stc_gravityVectorRegression Calculates the torques due to the robot
    % gravity effects.
    %
    %   [tau, Y] = Robot.stc_gravityVectorRegression(Q, Xtool) Calculates
    %       motor torques due to dynamics, including the end effector
    %   [tau, Y] = Robot.stc_gravityVectorRegression(Q) Calculates the
    %       robot torques without end effector effect.
    
    if nargin < 3 || isempty(Xtool)
        Xtool = [];
    end
    
    if nargin < 2 || isempty(Theta)
        Theta = DensoAlt5.stc_regressionTheta(true);
    end
    
    DOF = DensoAlt5.DOF;
    
    % Check inputs
    assert(size(Q, 1) == DOF && size(Q, 2) >= 1 && size(Q, 3) == 1, "Q must be a DOF x N matrix of joint angles, in radians!");
    
    
    if coder.target('matlab')
        
        try

            [tau, Y] = attemptCalc(Q, Theta, Xtool);

        catch e
            
            warning("Couldn't run codegen gravity regression script - likely robot is not built! Recommend building robot.");
            disp(e)

        end
        
    else
        
        [tau, Y] = attemptCalc(Q, Theta, Xtool);
        
    end
        
end

function [tau, Y] = attemptCalc(Q, Theta, Xtool)
    
    N = size(Q, 2);
    M = size(Theta, 1);
    DOF = DensoAlt5.DOF;
    Y = zeros(DOF, M, N);
    tau = zeros(DOF, N);
    
    for i = 1:N
        
        Y(:,:,i) = codegen_gravityRegressor(Q(:, i));

        tau(:, i) = Y(:, :, i)*Theta;

        if ~isempty(Xtool)
            G = codegen_EE_gravityVector(Q(:, i), Xtool);

            tau(:, i) = tau(:, i) + G;
        end
        
    end
    
end