function [tau] = predictTorquesSym(Robot, tau, vars, Qr, Qdr, Qddr, persist)
    % From Robot, symbolic expressions tau and scalars Q, Qd, Qdd, 
    % will output the torques in the joints. Equivalent to calling
    % Robot.inverseDynamics except the robot doesn't need to be made.
    
    persistent tauSFunc Iind symDHind mind rcind;
    
    N = size(Qr, 2);

    if nargin < 7 || isempty(persist) || ~persist || isempty(tauSFunc)

        Iind = Robot.symI~=0;
        symI = Robot.symI(Iind);

        symDH = Robot.symDH;
        symDH = symDH(:, [1 3]);
        symDHind = symDH(:)~= sym(0);
        symDH = symDH( symDHind );

        mind = Robot.symm~=0;
        symm = Robot.symm(mind);

        symg = Robot.symg(3);

        rcind = Robot.symrc~=0;
        symrc = Robot.symrc(rcind);

        Q = vars.Q;
        Qd = vars.Qd;
        Qdd = vars.Qdd;

        assumeAlso(Q, 'real')
        assumeAlso(Qd, 'real')
        assumeAlso(Qdd, 'real')

        tauSFunc = matlabFunction(tau, ...
            'Vars', {Q, Qd, Qdd, symDH, symI, symm, symg, symrc}');
    end

    tau = zeros(Robot.DOF, N);

    DH = Robot.DH(:, [1 3]);
    DH = DH(symDHind);
    
    I = Robot.inertiaTensor(Robot);
    I = I(Iind);
    
    m = Robot.m(mind);
    rc = Robot.rc(rcind);

    p = gcp('nocreate');
    if ~isempty(p) && false
        parfor i = 1:N
            tau(:, i) = tauSFunc(Qr(:, i), Qdr(:, i), Qddr(:, i), ...
                DH, I, m, utils.g, rc);
        end
    else
        for i = 1:N
            tau(:, i) = tauSFunc(Qr(:, i), Qdr(:, i), Qddr(:, i), ...
                DH, I, m, utils.g, rc);
        end
    end
    
end