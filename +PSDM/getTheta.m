function Theta = getTheta(DH_ext, X, g, E, P, Ntests)
    % GETTHETA For a given robot (SerialManipulator), calculates the Theta
    % matrix appropriate for the map and P given, based on the kinematic
    % and inertial properties of the robot.
    
    if nargin < 6 || isempty(Ntests)
        Ntests = round(size(P, 2) * 10);
    end

    [Q, Qd, Qdd, tau] = PSDM.genTestPoses(DH_ext, X, g, Ntests, 1);
    
    Yp = PSDM.genTermValues(Q, Qd, Qdd, E);
    
    Yb = utilities.blockprod(Yp, P);
    
    Yb_stack = utilities.vertStack(Yb);
    tau_stack = utilities.vertStack(tau, 2);
    
    Theta = Yb_stack \ tau_stack;
    
end