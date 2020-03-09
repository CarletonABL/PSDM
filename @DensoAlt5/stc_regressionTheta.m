function Theta = stc_regressionTheta(isGravity, calculate)
    % REGRESSIONTHETA Returns the regression parameter vector Theta for the
    % appropriate regression (gravity or full dynamics).
    % 
    %   T = Robot.stc_regressionTheta returns the full dynamics regression
    %   vector, calculated from cad
    %   T = Robot.stc_regressionTheta( isGravity ) allows the user to
    %   explicitely specify if you want the gravity regressor or the full
    %   regressor.
    %   T = Robot.stc_regressionTheta( isGravity, calculate ) allows the
    %   user to explicitely specify if Theta should be taken directly from
    %   the "Theta" or "ThetaGrav" property on the object, or calculated.
    %
    % If the robot has a property "Theta" on it (or a property "ThetaGrav"
    % for gravity), then it will be returned.
    %
    % See also SerialManipulator.regressionDynamics,
    % SerialManipulator.regressionGravity. 
    
    if nargin < 1
        isGravity = false;
    end
    
    if nargin < 2
        calculate = true;
    end
    
    if ~calculate
        % Try to access properties on robot. If fails, just calculate
       
        if isGravity
            try
                Theta = DensoAlt5.ThetaGrav;
            catch
                warning("Could not access ThetaGrav property on robot. Check definition. Using calculated value instead");
                Theta = codegen_gravityTheta(DensoAlt5.Ilist, DensoAlt5.m, DensoAlt5.rc);
            end
        else
            try
                Theta = DensoAlt5.Theta;
            catch
                warning("Could not access Theta property on robot. Check definition. Using calculated value instead");
                Theta = codegen_dynamicsTheta(DensoAlt5.Ilist, DensoAlt5.m, DensoAlt5.rc);
            end
        end
        
    else
        % Just calculate
        
        if isGravity
            Theta = codegen_gravityTheta(DensoAlt5.Ilist, DensoAlt5.m, DensoAlt5.rc);
        else
            Theta = codegen_dynamicsTheta(DensoAlt5.Ilist, DensoAlt5.m, DensoAlt5.rc);
        end
        
    end
    
end