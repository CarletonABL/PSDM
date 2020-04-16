function Theta = X2Theta(DH_ext, X, g, E, P, Ntests)
    % X2Theta For a given robot defined by DH_ext and with known 
    % inertial properties X, calculates the Theta
    % matrix appropriate for the map and P given, based on the kinematic
    % and inertial properties of the robot.
    
    if nargin < 6 || isempty(Ntests)
        Ntests = round(size(P, 2) * 10);
    end

    % Make some samples to test
    [Q, Qd, Qdd, tau] = PSDM.generateSamples(DH_ext, X, g, Ntests, 1);
    
    % Generate Yp and Yb
    Yp = PSDM.generateYp(Q, Qd, Qdd, E);
    Yb = utilities.blockprod(Yp, P);
    
    % Stack DOF of robot along regression dimension, then solve for theta
    Yb_stack = utilities.vertStack(Yb);
    tau_stack = utilities.vertStack(tau, 2);
    Theta = utilities.mldivide2(Yb_stack, tau_stack);
    
end