function [tauTAll, fTAll, extraAll] = ...
    inverseDynamicsNewton(DH_ext, Xlist, Q, Qd, Qdd, outputFrame, g, Rbase)
    % INVERSEDYNAMICSNEWTON Computes the reaction torques and forces in
    % each joint due to a motion defined by Q (joint variables), Qd (joint
    % speeds, and Qdd (joint accelerations). All mass, center of gravity
    % and inertia information is taken from the robot object, so this must
    % be specified in the class.
    %
    % [tau, f] = inverseDynamicsNewton(DH_ext, Xlist, Q, Qd, Qdd)
    %
    % [tau, f] = inverseDynamicsNewton(props, Q, Qd, Qdd, Xtool)
    %   additionally adds end effector effect.
    %
    % [tau, f] = inverseDynamicsNewton(props, Q, Qd, Qdd, ___, outputFrame)
    %   allows one to choose which frame the forces should be resolved in.
    %
    % [tau, f] = inverseDynamicsNewton(props, Q, Qd, Qdd, ___, outputFrame, g, Rbase)
    %   Additionally specifies the gravity direction vector and the base
    %   rotation of the robot.
    %
    % [tau, f, extra] = inverseDynamicsNewton(____) additionally outputs a
    %   structure with the following variables: alpha, omega (w), ae, ac.
    %   
    % Inputs:
    %   - DH_ext: a DOFx4-6 array of the DH params in the following order: 
    %             [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
    %              :       :        :       :         :         :     
    %              an   alpha_n    d_n   theta_n    lt_n    q_sign_n];
    %           lt stands for "Link types", and should be true if joint is a
    %           prismatic joint, false otherwise. If column 5 is missing, will
    %           assume revolute (0).
    %           q_sign is a link direction number (-1 or 1). Allows you to
    %           change the sign of the joint variable. If column 6 is
    %           missing, will assume 1.
    %   - X:    A DOF x 10 matrix of the mass properties of the
    %           manipulator:
    %           [m_1  rcx_1  rcy_1  rcz_1  Ixx_1  Iyy_1  Izz_1  Ixy_1  Ixz_1  Iyz_1;
    %             :     :      :      :       :     :      :      :     :       :
    %           [m_n  rcx_n  rcy_n  rcz_n  Ixx_n  Iyy_n  Izz_n  Ixy_n  Ixz_n  Iyz_n];
    %           If you include a DOF+1 row to this, it will assume that
    %           this is the tool effect and combine it into the
    %           equations.
    %   - Q, Qd, Qdd [DOF x 1]: Joint variables of the motion (radians for
    %           angles!).
    %   - outputFrame [int8]: The ID of the frame you want to output the
    %          forces in.
    %            * 1:  Forces in the joint are resolved in the frame of the
    %                  following link (i+1) [default].
    %            * 0:  Forces in joint are resolved in the previous frame. This
    %                  can be useful since the "z" component of torque/force will
    %                  be the same as the the joint torque.
    %           * -1: Forces are resolved in the world frame.
    %   - g:  A 3x1 vector unit vector indicating the direction of the
    %         gravity. Will be scaled by the value of gravity returned
    %         by utils.g.
    %         If this vector is set to identically [0 0 0], then gravity
    %         will be ignored.
    %         Default: [0 0 1].
    %   - Rbase: Optionally add a 3x3 orthonormal rotation to the base
    %         the robot (note, this is equivalent to just rotating the
    %         gravity vector).
    %         Default: eye(3).
    %   
    % Outputs:
    %   - tau [3 x DOF]: The reaction torques in each joint.
    %   - f [3 x DOF]: The reaction forces in each joint.
    %   - extra (struct): Additional structure with rotation velocity,
    %     acceleration, and linear acceleration of the ends of links and
    %     center of gravities. All quantities are in standard units
    %     (rad/s^2, rad/s, m/s^2). All dimensions are [3 x DOF].
    %
    % NOTE! Currently, this script only works on revolute joint robots.
    % Also todo, add easy way to add Fext to the computation (since you get
    % it essentially free). Should also add functionality to simultanioulsy
    % compute multiple joints at once (not done yet).
    %
    % Code has been tested on revolute joint robots only, through
    % comparison with Simulink multibody and with the Lagrangian
    % closed-form equations.
    %
    % See also: SerialManipulator.inverseDynamicsNewton.

    %% Parse arguments

    % Parse DH parameters
    assert(size(DH_ext, 1) >= 1 && size(DH_ext, 2) >= 4, ...
        "DH is the wrong size!");

    DH = DH_ext(:, 1:4);
    DOF = size(DH, 1);
    N = size(Q, 2);

    if size(DH_ext, 2) >= 5
        lt = logical(DH_ext(:, 5));
    else
        lt = zeros(DOF, 1, 'logical');
    end

    if size(DH_ext, 2) >= 6
        qsign = sign(DH_ext(:, 6));
    else
        qsign = ones(DOF, 1);
    end

    assert(all(lt == 0), ...
        ['This script currently only works for revolute joint robots ', ...
         'only. Feel free to extend it to the general case!']);
    
    % Parse Xlist
    assert( (size(Xlist, 1) == DOF || size(Xlist, 1) == DOF+1) && size(Xlist, 2) == 10, ...
        "Xlist is the wrong size!");
    
    if size(Xlist, 1) == DOF+1
        Xtool = Xlist(DOF+1, :);
        Xrobot = Xlist(1:DOF, :);
    else
        Xtool = zeros(1, 10);
        Xrobot = Xlist;
    end

    m = Xrobot(:, 1)';
    rc = Xrobot(:, 2:4)';
    I = SerialManipulator.inertiaTensor(Xrobot(:, 5:10));

    % If Xtool is non-zero, then combine its inertia with that of the last
    % link, then proceed normally.
    if any(abs(Xtool) > eps)
        Xend = RU.combineBodyInertias(Xrobot(DOF, :), Xtool);
        m(end) = Xend(1);
        rc(:, end) = Xend(1, 2:4)';
        I(:, :, end) = SerialManipulator.inertiaTensor(Xend(1, 5:10));
    end

    % Check Q vectors
    assert( all(size(Q) == size(Qd)) && all(size(Q) == size(Qdd)), ...
        "Joint variables sizes must match!");

    % Parse outputFrame
    if nargin < 6 || isempty(outputFrame)
        outputFrame = int8(1);
    end

    % Parse g and Rbase
    if nargin < 7 || isempty(g)
        g = [0 0 1];
    end
    if nargin < 8 || isempty(Rbase)
        Rbase = eye(3);
    end
    
    
    %% Check if MEX exists, if so, run that
    if coder.target('matlab')
        try
            [tauTAll, fTAll, extraAll] = ...
                RU.inverseDynamicsNewton_mex(double(DH_ext), double(Xlist), double(Q), double(Qd), double(Qdd), int8(outputFrame), double(g), double(Rbase));
            return;
        catch
            warning("Could not run mex file! Likely need to compile RU toolbox with RU.make.");
        end
    end
    

    %% Start Main Processing

    % Build properties matrix
    props.DOF = DOF;
    props.DH = DH;
    props.rc = rc;
    props.m = m;
    props.g = -g * utils.g;
    props.I = I;
    props.lt = lt; % logical zero if revolute, logical 1 if prismatic
    props.qsign =qsign;
    props.Rbase = Rbase;   
    
    % Pre-init variables
    tauTAll = zeros(3, DOF, N);
    fTAll = zeros(3, DOF, N);
    extraAll = repmat(struct('alpha', nan(3, DOF), ...
                              'w', nan(3, DOF), ...
                              'ae', nan(3, DOF), ...
                              'ac', nan(3, DOF)), ...
                      [1 N]);
    
    % Run loop in parallel if possible.
    % Use parallel if in matlab and parallel pool open, or if using matlab
    % coder
    if coder.target('matlab')
        doPar = ~isempty(gcp('nocreate'));
    else
        doPar = true;
    end
    
    if doPar
     
        % Parfor loop
        parfor j = 1:N
            [tauTAll(:, :, j), fTAll(:, :, j), extraAll(j)] = ...
                mainNELoop(props, Q(:, j), Qd(:, j), Qdd(:, j), outputFrame);
        end 
        
    else
        
        % Regular for loop
        for j = 1:N
            [tauTAll(:, :, j), fTAll(:, :, j), extraAll(j)] = ...
                mainNELoop(props, Q(:, j), Qd(:, j), Qdd(:, j), outputFrame);
        end
        
    end
        
    
end

%% Main Newton Euler Function

function [tauT, fT, extra] = mainNELoop(props, Q, Qd, Qdd, outputFrame)
    % Computes tau, f, and extra variables from props (property list of
    % manipulator), Q, Qd, Qdd (joint variables) and outputframe.

    extra = struct;
    DOF = props.DOF;

    % Get DH parameters (in current pose)
    DH = props.DH;
    DH(:, 3) = DH(:, 3) + Q.*(props.lt).*props.qsign;
    DH(:, 4) = DH(:, 4) + Q.*(~props.lt).*props.qsign;

    % Initiate matrices to store results of forward and reverse recursion.
    % Each column is the value of frame i-1. First row is all zeros.
    w = zeros(3, DOF+1);
    alpha = zeros(3, DOF+1);
    ac = zeros(3, DOF+1);
    ae = zeros(3, DOF+1);

    f = zeros(3, DOF+1);
    tau = zeros(3, DOF+1);

    % To include EE effect, set terminal conditions of f and tau to nonzero
    % values

    % Pre-init some values which are stored along the way
    % R is the rotation matrices from world frame to the current frame.
    % Its first entry is just the base rotation.
    % The last entry is the identity (last frame to tool), but we leave it
    % in to make coding easier later.
    R = cat(3, props.Rbase, ...
               zeros(3, 3, DOF+1));

    % Rdiff is the rotation matrix of the transformation between frames
    Rdiff = cat(3, zeros(3, 3, DOF), ...
                   eye(3));

    % odiff is the vector between frames
    odiff = zeros(3, DOF);

    % Forward recursion 1 -> DOF
    for i = 1:DOF

        % Establish variables for this frame.
        % Naming convension: XX_i   -> XX of frame i
        %                    XX_im1 -> XX of frame i - 1
        %                    XX_ip1 -> XX of frame i + 1

        w_im1 = w(:, i);
        alpha_im1 = alpha(:, i);
        ae_im1 = ae(:, i);
        R_im1 = R(:, :, i);
        is_pris = props.lt(i);
        is_rev = ~props.lt(i);

        % Rotation matrix of the ith frame is just the standard DH
        % transform
        Rdiff_i = makeDHRot(DH(i, 4), DH(i, 2));
        Ri = R_im1*Rdiff_i;

        % The axis of the previous rotation is just the 3rd column of that
        % rotation matrix
        z_im1 = R_im1(:, 3);

        % Vector from o_i to o_i+1
        r_i_ip1 = utils.rotx(DH(i, 2))'*[DH(i, 1); 0; DH(i, 3)];

        % Vector from o_i to o_ci
        r_i_ci = r_i_ip1 + props.rc(:, i);

        % Spong, 2nd edition, p278, eq. 7.150
        b_i = Ri' * z_im1; 

        % Spong, 2nd edition, p278, eq. 7.149
        w_i = Rdiff_i' * w_im1 + is_rev * b_i*Qd(i);

        % Spong, 2nd edition, p278, eq. 7.153
        alpha_i = Rdiff_i'*alpha_im1 + ...
            is_rev * b_i * Qdd(i) + ...
            is_rev * cross( w_i, b_i*Qd(i) );

        % Spong, 2nd edition, p279, eq. 7.159
        % This equation needs to be updated to allow for prismatic joints
        ae_i = Rdiff_i' * ae_im1 + ...
            cross( alpha_i, r_i_ip1 ) + ...
            cross( w_i, cross(w_i, r_i_ip1) );

        % Spong, 2nd edition, p279, eq. 7.158
        % This equation needs to be updated to allow for prismatic joints
        ac_i = Rdiff_i' * ae_im1 + ...
            cross( alpha_i, r_i_ci ) + ...
            cross( w_i, cross(w_i, r_i_ci) );

        % Store values in arrays
        w(:, i+1) = w_i;
        alpha(:, i+1) = alpha_i;
        ae(:, i+1) = ae_i;
        ac(:, i+1) = ac_i;
        R(:, :, i+1) = Ri;
        Rdiff(:, :, i) = Rdiff_i;
        odiff(:, i) = r_i_ip1;
    end

    % Backwards recursion DOF -> 1
    for i = DOF:-1:1

        % Retrieve the variables for this iteration.
        R_i = R(:, :, i+1);
        Rdiff_ip1 = Rdiff(:, :, i+1);        
        f_ip1 = f(:, i+1);
        tau_ip1 = tau(:, i+1);
        alpha_i = alpha(:, i+1);
        w_i = w(:, i+1);
        ac_i = ac(:, i+1);
        m_i = props.m(i);
        I_i = props.I(:, :, i);

        % Gravity vector in current frame
        g_i = R_i' * props.g;

        % Vector from o_i to o_ci
        r_i_ci = odiff(:, i) + props.rc(:, i);

        % Vector from o_i+1 -> o_ci
        r_ip1_ci = props.rc(:, i);

        % Spong, 2nd edition, p279, eq. 7.146
        f_i = Rdiff_ip1 * f_ip1 ...
            + m_i * ac_i ...
            - m_i * g_i;

        % Spong, 2nd edition, p279, eq. 7.147
        % (note, there is a mistake in the book!)
        tau_i = Rdiff_ip1 * tau_ip1 ...
            - cross(f_i, r_i_ci) ...
            + cross( (Rdiff_ip1 * f_ip1), r_ip1_ci) ...
            + I_i * alpha_i ...
            + cross( w_i, I_i * w_i );

        % Store in arrays
        f(:, i) = f_i;
        tau(:, i) = tau_i;

    end

    % Store additional variable in structure for output, if necessary.
    extra.alpha = alpha(:, 2:end);
    extra.w = w(:, 2:end);
    extra.ae = ae(:, 2:end);
    extra.ac = ac(:, 2:end);

    tauT = zeros(3, DOF);
    fT = zeros(3, DOF);

    % Put back everything into the correct frame (depending on what frame
    % was requested
    switch outputFrame

        case int8(1)
            % Nothing to do, just trim down variables
            fT(:, :) = f(:, 1:DOF);
            tauT(:, :) = tau(:, 1:DOF);
            return;
            
        case int8(0)
            % Need to rewrite it backwards one frame
            Radj = Rdiff;

        case int8(-1)
            % Base frame (note, not tested! Should be
            % correct though...)
            Radj = R(:, :, 2:end);

        otherwise
            error("Invalid outputFrame! Must be -1, 0 or 1");

    end

    % If we get here, need to adjust the frame.
    for i = 1:DOF
        % For all quantities, multiply by the appropriate rotation matrix.
        fT(:, i) = Radj(:, :, i) * f(:, i);
        tauT(:, i) = Radj(:, :, i) * tau(:, i);

        extra.alpha(:, i) = Radj(:, :, i) * alpha(:, i);
        extra.w(:, i) = Radj(:, :, i) * w(:, i);
        extra.ae(:, i) = Radj(:, :, i) * ae(:, i);
        extra.ac(:, i) = Radj(:, :, i) * ac(:, i);

    end
    
end


function R = makeDHRot(theta, alpha)
    % MAKEDHROT Makes the rotation matrix from frame i -> i+1 according
    % to the DH convention

    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    
    R = [ct, -st*ca, st*sa;
         st, ct*ca, -ct*sa;
         0, sa, ca];
     
end