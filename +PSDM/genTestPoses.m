function [Q, Qd, Qdd, tau] = genTestPoses(DH_ext, X, g_in, Nq, Nt, type_in)
    % GENTESTPOSES: Generates joint angles and corresponding torques for Nq
    % different (random) joint states, and Nt different inertial
    % parameters.
    %
    % [Q, Qd, Qdd] = genTestPoses(DH_ext, X, Nq, Nt, type)
    % [Q, Qd, Qdd] = genTestPoses(DOF, N, type
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
    if isempty(g_in)
        g = zeros(3, 1);
    else
        g = g_in(1:3, 1);
        coder.varsize('g', [3 1], [0 0]);
    end
    
    %% Try to run mex
    
    % Run mex, if possible
    if coder.target('matlab')
        
        try
            if nargout > 3
                [Q, Qd, Qdd, tau] = PSDM.genTestPoses_mex(DH_ext, X, g, Nq, Nt, {type, joints});
            else
                [Q, Qd, Qdd] = PSDM.genTestPoses_mex(DH_ext, X, g, Nq, Nt, {type, joints});
            end
            
            return;
            
        catch
            
            warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
        
        end
    end

    %% Start Function
    
    % Generate DH and Xlist params
    DOF = size(DH_ext, 1);
    Xlist = getRobotInertiaProps(X, Nt);
    
    % If gravity
    if strcmp(type, 'gravity')
        
        % Check that a proper gravity vector was given
        assert(sum(g.^2) == 1, "Must give a unit vector for gravity direction, for gravity ID!");
        
        % Random joint positions, no velocity or acceleration terms
        % Q = (rand(DOF, Nq) * 2 - 1)*pi;
        Q = generateRandomPose(DOF, Nq);
        Qd = zeros(size(Q));
        Qdd = zeros(size(Q));
        
        % If torque is requested give that
        if nargout > 3
            
            tau = zeros(Nq, DOF, Nt);
            
            for j = 1:Nt
               tauAll = PSDM.inverseDynamicsNewton(DH_ext, Xlist(:, :, j), Q, Qd, Qdd, int8(0), g);
               tau(:, :, j) = permute(tauAll(3, :, :), [3 1 2]);
            end
            
        end
        
    elseif strcmp(type, 'accel')
        
        % Random joint positions, no velocities
        % Q = (rand(DOF, Nq) * 2 - 1)*pi;
        Q = generateRandomPose(DOF, Nq);
        Qd = zeros(size(Q));
        
        % Different joint accels depending on if we are isolating joints or
        % not
        if ~isempty(joints)
            
            Qdd = zeros(size(Q));
            Qdd(joints, :) = (rand(numel(joints), Nq) * 2 - 1) * pi * 1;
            
        else
            
            Qdd = (rand(DOF, Nq) * 2 - 1) * pi * 1;
            
        end
        
        % Give torque, if requested
        if nargout > 3
            
            tau = zeros(Nq, DOF, Nt);
            for j = 1:Nt
                
               tauAll = PSDM.inverseDynamicsNewton(DH_ext, Xlist(:, :, j), Q, Qd, Qdd, int8(0), [0 0 0]');
               tau(:, :, j) = permute(tauAll(3, :, :), [3 1 2]);
               
            end
            
        end
        
    elseif strcmp(type, 'velocity') || ...
           strcmp('centripital', type) || ...
           strcmp(type, 'coriolis')
        
        % Random joint positions, no accelerations
        Q = generateRandomPose(DOF, Nq);
        % Q = (rand(DOF, Nq) * 2 - 1)*pi;
        Qdd = zeros(size(Q));
        
        % Different joint accels depending on if we are isolating joints or
        % not
        if ~isempty(joints)
            
            Qd = zeros(size(Q));
            Qd(joints, :) = (rand(numel(joints), Nq) * 2 - 1) * pi * 5;
            
        else
            
            Qd = (rand(DOF, Nq) * 2 - 1) * pi * 5;
            
        end
        
        % Give torque, if possible
        if nargout > 3
            
            tau = zeros(Nq, DOF, Nt);
            for j = 1:Nt
            
               tauAll = PSDM.inverseDynamicsNewton(DH_ext(:, :), Xlist(:, :, j), Q, Qd, Qdd, int8(0), [0 0 0]');
               
               if strcmp(type, 'coriolis')
                    Qd_cent1 = Qd; Qd_cent1(joints(2), :) = zeros(1, Nq);
                    Qd_cent2 = Qd; Qd_cent2(joints(1), :) = zeros(1, Nq);
                    tauCent1 = PSDM.inverseDynamicsNewton(DH_ext(:, :), Xlist(:, :, j), Q, Qd_cent1, Qdd, int8(0), [0 0 0]');
                    tauCent2 = PSDM.inverseDynamicsNewton(DH_ext(:, :), Xlist(:, :, j), Q, Qd_cent2, Qdd, int8(0), [0 0 0]');
                    tauAll = tauAll - tauCent1 - tauCent2;
               end
               
               tau(:, :, j) = permute(tauAll(3, :, :), [3 1 2]);
            end
            
        end % End of "give torque"
        
    elseif strcmp(type, 'all')
        
        % Check that a proper gravity vector was given
        assert(sum(g.^2) == 1, "Must give a unit vector for gravity direction, for gravity ID!");
        
        % Just random joints. No gravity comp
        % Q = (rand(DOF, Nq) * 2 - 1)*pi;
        Q = generateRandomPose(DOF, Nq);
        Qd = (rand(DOF, Nq) * 2 - 1)*pi;
        Qdd = (rand(DOF, Nq) * 2 - 1)*pi;
        
        % Give torque, if needed
        if nargout > 3
            
            tau = zeros(Nq, DOF, Nt);
            for j = 1:Nt
               tauAll = PSDM.inverseDynamicsNewton(DH_ext, Xlist(:, :, j), Q, Qd, Qdd, int8(0), g);
               tau(:, :, j) = permute(tauAll(3, :, :), [3 1 2]);
            end
            
        end
        
    else % End of all type
        
        error("Unknown request type");
    
    end
    
end

function Qout = generateRandomPose(DOF, Nq)
    % Generates random pose. Can edit this function to try to generate
    % better sampling.
    
    Qout = (rand(DOF, Nq) * 2 - 1)*pi;
    
end

function [Xlist] = getRobotInertiaProps(X, Nt)
    % GETROBOTINERTIAPROPS: Generates a set of Nt random inertial
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