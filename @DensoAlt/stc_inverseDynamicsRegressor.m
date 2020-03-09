function [tau, Y] = stc_inverseDynamicsRegressor(Q, Qd, Qdd, Xtool)

    % STC_REGRESSIONDYNAMICS Calculates the torques due to the robot

    % dynamics using the regression formulae.    

    %

    %   tau = Robot.stc_inverseDynamicsRegressor(Q, Qd, Qdd, Xtool) Calculates

    %       motor torques due to dynamics, including the end effector

    %   tau = Robot.stc_inverseDynamicsRegressor(Q, Qd, Qdd) Calculates the

    %       robot torques without end effector effect.

    %   [tau, Y] = Robot.stc_inverseDynamicsRegressor(___) additionally returns

    %       the regression information matrix Y.

    

    DOF = DensoAlt.DOF;

    

    if nargin < 5 || isempty(Xtool)

        Xtool = [];

    end

    

    assert(size(Q, 1) == DOF && size(Q, 2) >= 1 && size(Q, 3) == 1, "Q must be a DOF x N matrix of joint angles, in radians!");

    assert(all(size(Q) == size(Qd)) && all(size(Q) == size(Qdd)), "Q, Qd, and Qdd must all have the same dimensions.");

    

    T = DensoAlt.stc_regressionTheta(false);

    

    if coder.target('matlab')

        

        try



            [tau, Y] = attemptCalc(Q, Qd, Qdd, T, Xtool);



        catch



            warning("Couldn't run codegen dynamics - likely robot is not built! Recommend building robot.");



        end

        

    else

        

        [tau, Y] = attemptCalc(Q, Qd, Qdd, T, Xtool);

        

    end

        

end



function [tau, Y] = attemptCalc(Q, Qd, Qdd, T, Xtool)

    

    N = size(Q, 2);

    M = size(T, 1);

    DOF = DensoAlt.DOF;

    Y = zeros(DOF, M, N);

    tau = zeros(DOF, N);

    

    for i = 1:N



        Y(:,:,i) = codegen_dynamicsRegressor(Q(:, i), Qd(:, i), Qdd(:, i));

        tau(:, i) = Y(:, :, i)*T;



        if ~isempty(Xtool)

            D = codegen_EE_massMatrix(Q(:, i), Xtool);

            C = codegen_EE_coriolisMatrix(Q(:, i), Qd(:, i), Xtool);

            G = codegen_EE_gravityVector(Q(:, i), Xtool);



            tau(:, i) = tau(:, i) + D*Qdd(:, i) + C*Qd(:, i)+ G;

        end

    

    end

    

end