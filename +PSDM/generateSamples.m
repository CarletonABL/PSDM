function [Q, Qd, Qdd, tau] = generateSamples(DH_ext, X, g_in, Nq, Nt, type_in)
    % GENERATESAMPLES: Generates joint angles and corresponding torques for Nq
    % different (random) joint states, and Nt different inertial
    % parameters.
    %
    % [Q, Qd, Qdd] = generateSamples(DH_ext, X, g_in, Nq, Nt, type)
    %
    %   INPUTS:
    %       -DH_ext, X, g: As per run id.
    %       -Nq : Number of joint positions to generate
    %       -Nt : Number of inertial parameters to generate at.
    %       -type: string or cell array.
    %           type{1} = 'all', 'accel', 'gravity', 'velocity',
    %               'coriolis', or 'centrifugal'.
    %           type{2} = joint indices to target (used for velocity and
    %               accel types).
    
    %% Parse arguments
    
    % Parse type
    if nargin < 6 || isempty(type_in)
        type = 'all';
        joints = [];
    else
    
        if isa(type_in, 'cell')
            joints = type_in{2};
            type = type_in{1};
        else
            type = type_in;
            joints = [];
        end
        
    end
    
    % Parse gravity vector
    if isempty(g_in)
        g = zeros(3, 1);
    else
        g = g_in(1:3, 1);
        assert(~any(strcmp({type, type}, {'gravity', 'all'})) || abs(sum(g.^2) - 1) < 1e-2, "Must give a unit vector for gravity direction!");
        coder.varsize('g', [3 1], [0 0]);
    end
    
    % Run mex, if possible
    c = PSDM.config;
    if coder.target('matlab') && c.allow_mex
        try
            if nargout > 3
                [Q, Qd, Qdd, tau] = PSDM.generateSamples_mex(DH_ext, X, g, Nq, Nt, {type, joints});
            else
                [Q, Qd, Qdd] = PSDM.generateSamples_mex(DH_ext, X, g, Nq, Nt, {type, joints});
            end
            
            return;
            
        catch
            
            warning("PSDM is not compiled! PSDM.generateSamples will run slowly without compilation. Recommend running PSDM.make");
        
        end
    end

    %% Start Function
    
    DOF = size(DH_ext, 1);
    
    % Different behaviour depending on ID type
    switch type
        
        case 'gravity'
            [Q, Qd, Qdd] = generateSamplesGravity(DOF, Nq);
        
        case 'accel'
            [Q, Qd, Qdd] = generateSamplesAccel(DOF, Nq, joints);
        
        case {'velocity', 'centripital', 'coriolis'}
            [Q, Qd, Qdd] = generateSampleVelocity(DOF, Nq, joints);
        
        case 'all'
            [Q, Qd, Qdd] = generateSampleAll(DOF, Nq);
        
        otherwise
            error("Unknown request type");
    
    end
    
    % Generate torques, if needed
    if nargout > 3
        % Generate list of random inertial properties for sampling
        Xlist = getRobotInertiaProps(X, Nt);
        
        % Get torques
        tau = generateTorques(Q, Qd, Qdd, joints, DH_ext, Xlist, g);
    end
    
end

function [Q, Qd, Qdd] = generateSamplesGravity(DOF, Nq)
    %% Gets samples for a gravity model ID.

    % Random joint positions, no velocity or acceleration terms
    Q = generateRandomPose(DOF, Nq);
    Qd = zeros(size(Q));
    Qdd = zeros(size(Q));


end


function [Q, Qd, Qdd] = generateSamplesAccel(DOF, Nq, joints)
    %% Get samples for an acceleration ID.

    % Random joint positions, no velocities
    Q = generateRandomPose(DOF, Nq);
    Qd = zeros(size(Q));

    % Different joint accels depending on if we are isolating joints or
    % not
    s = 1;
    if ~isempty(joints)

        Qdd = zeros(size(Q));
        Qdd(joints, :) = (rand(numel(joints), Nq) * s - s/2);
        Qdd = sign(Qdd) .* (abs(Qdd) + s/10);

    else
        
        Qdd = rand(DOF, Nq) * s - s/2;
        Qdd = sign(Qdd) .* (abs(Qdd) + s/10);

    end
        
end

function [Q, Qd, Qdd] = generateSampleVelocity(DOF, Nq, joints)
    %% Get samples for a velocity ID
    
    % Random joint positions, no accelerations
    Q = generateRandomPose(DOF, Nq);
    Qdd = zeros(size(Q));

    % Different joint accels depending on if we are isolating joints or
    % not
    s = 1;
    if ~isempty(joints)

        Qd = zeros(size(Q));
        
        Qd(joints, :) = (rand(numel(joints), Nq) * s - s/2);
        Qd = sign(Qd) .* (abs(Qd) + s/10);
        
    else

        Qd = rand(DOF, Nq) * s - s/2;
        Qd = sign(Qd) .* (abs(Qd) + s/10);

    end

end

function [Q, Qd, Qdd] = generateSampleAll(DOF, Nq)
    %% Generate samples for "All" case.
    
    % Check that a proper gravity vector was given
        
        % Just random joints. No gravity comp
        Q = generateRandomPose(DOF, Nq);
        s = 1;
        Qd = rand(DOF, Nq) * s - s/2;
        Qd = sign(Qd) .* (abs(Qd) + s/10);
        Qdd = rand(DOF, Nq) * s - s/2;
        Qdd = sign(Qdd) .* (abs(Qdd) + s/10);

end


function tau = generateTorques(Q, Qd, Qdd, joints, DH_ext, Xlist, g)
    %% Generate torques

    Nt = size(Xlist, 3);
    Nq = size(Q, 2);
    DOF = size(DH_ext, 1);
    
    tau = zeros(Nq, DOF, Nt);
    isCoriolis = numel(joints) == 2;
    
    for j = 1:Nt
        
       tau_j = PSDM.inverseDynamicsNewton(DH_ext, Xlist(:, :, j), Q, Qd, Qdd, int8(2), g)';
       
       % For a coriolis ID, need to subtract away the centrifugal terms
        if isCoriolis
            Qd_cent1 = Qd; Qd_cent1(joints(2), :) = zeros(1, Nq);
            Qd_cent2 = Qd; Qd_cent2(joints(1), :) = zeros(1, Nq);
            tauCent1 = PSDM.inverseDynamicsNewton(DH_ext(:, :), Xlist(:, :, j), Q, Qd_cent1, Qdd, int8(2), [0 0 0]')';
            tauCent2 = PSDM.inverseDynamicsNewton(DH_ext(:, :), Xlist(:, :, j), Q, Qd_cent2, Qdd, int8(2), [0 0 0]')';
            tau_j = tau_j - tauCent1 - tauCent2;
        end
        
        tau(:, :, j) = tau_j;
        
    end
    
end


function Qout = generateRandomPose(DOF, Nq)
    %% Function for generating random joint poses.
    % Could potentially be optimized to get better condition on matrix
    % inversions.
    
    Qout = (rand(DOF, Nq) * 2 - 1)*pi;
    
end


function Xlist = getRobotInertiaProps(X, Nt)
    %% GETROBOTINERTIAPROPS: Generates a set of Nt random inertial
    % properties.
    
    DOF = size(X, 1);

    if Nt == 1
        % If Nt == 1, use robot props, but perturbe slightly
        Xlist = X;% .* ((rand(DOF, 10)-.5)*.1 + 1);
        
    else
        
        % Make Nt random properties
        Xlist = rand(DOF, 10, Nt);
        
        % Some can be negative
        Xlist(:, [2 3 4 8 9 10], :) = Xlist(:, [2 3 4 8 9 10], :) * 2 - 1;
        
        % Replace zero terms on robot with zero terms
        Xlist = Xlist .* repmat((abs(X(1:DOF, :)) > eps), [1 1 Nt]);
        
    end

end