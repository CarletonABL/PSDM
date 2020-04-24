function Ym = getSetYm(DH_ext, type, Up_acc_in)
    % GETSETYM Returns the full search space of projected acceleration
    % functions for a manipulator.
    %
    % Inputs:
    %   -DH_ext: DOF x 6 matrix of DH parameters in order
    %        [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
    %          :      :        :       :         :         :     
    %         a_n  alpha_n    d_n   theta_n    lt_n    q_sign_n];
    %   -type: The type of accelerations desired. Options are: 'gravity',
    %       'velocity', 'coriolis', 'centrifugal', 'joint' or 'all'.
    %   -Up_acc: A (3*DOF)xM or (5*DOF)xM matrix representing either the
    %       Upsilon or the Ym matrix of the acceleration functions. Will be
    %       used to reduce the set of possible centrifugal terms.
    %
    % Output:
    %   -Ym: The functions in the set Ym for the manipulator, represented
    %       as a (5*DOF)xM integer matrix of exponents of the functions:
    %       [Q; sin(Q); cos(Q); Qd; Qdd]
    
    DOF = size(DH_ext, 1);
    
    Up1 = PSDM.getSetUpsilon(DH_ext, uint8(1));
    Up2 = PSDM.getSetUpsilon(DH_ext, uint8(2));
    
    
    % Velocity upsilon can be calculated from acceration terms, if given,
    % or just from Up2.
    if nargin > 2 && ~isempty(Up_acc_in)
        Up_vel = parseUp_acc(Up_acc_in, DH_ext);
    else
        Up_vel = Up2;
    end
    
    % Get accelerations
    Agrav = PSDM.getSetA(DOF, 'gravity');
    Acent = PSDM.getSetA(DOF, 'centrifugal');
    Acor = PSDM.getSetA(DOF, 'coriolis');
    Ajoint = PSDM.getSetA(DOF, 'joint');
    
    % Make the Y terms
    Ygrav = makeY(Up1, Agrav);
    Ycent = makeY(Up_vel, Acent);
    Ycor = makeY(Up_vel, Acor);
    Yjoint = makeY(Up2, Ajoint);
    
    % Output correct type
    switch type
        case 'gravity'
            Ym = Ygrav;
        case 'coriolis'
            Ym = simplifyY_vel( Ycor, DH_ext );
        case 'centrifugal'
            Ym = simplifyY_vel( Ycent, DH_ext );
        case 'joint'
            Ym = simplifyY_accel( Yjoint, DH_ext );
        case 'velocity'
            Ym = simplifyY_vel( horzcat(Ycor, Ycent ) , DH_ext );
        case 'all'
            Ym = horzcat(Ygrav, Ycent, Ycor, Yjoint);
        otherwise
            error("Invalid type given!");
    end
    
end

function Y = makeY(Up, A)
    % MAKEY Combines an upsilon set Up and an acceleration set A into a set
    % Y
    
    Macc = size(A, 2);
    Mup = size(Up, 2);
    
    Y = vertcat( repmat(Up, [1, Macc]), ...
                 repelem(A, 1, Mup) );

end

function Ym_trim = simplifyY_accel( Ym, DH_ext )
    % We can filter out all functions of joint 1 for Upsilon_2 when dealing
    % with acceleration functions.
    
    DOF = size(DH_ext, 1);
    m = size(Ym, 2);
    mask = ones(1, m, 'logical');
    
    removeMask = any(Ym( horzcat( 1, DOF+1, 2*DOF+1 ), :) > 0, 1);
    mask(removeMask) = false;
    
    Ym_trim = Ym(:, mask);

end


function Ym_trim = simplifyY_vel( Ym, DH_ext )
    % We can filter out all centrifugal terms which have elements of lower
    % joint angles trig functions of order greater than 1, same thing for
    % coriolis terms
    
    DOF = size(DH_ext, 1);
    m = size(Ym, 2);
    mask = ones(1, m, 'logical');
    
    for i = 1:DOF
        centJointMask = Ym(3*DOF + i, :) == uint8(2);
        trigMask = any( Ym(1:i, :) + Ym((1:i) + DOF, :) + Ym((1:i) + 2*DOF, :) > uint8(1), 1);
        removeMask = all( vertcat(centJointMask, trigMask), 1);
        mask(removeMask) = false;
    end
    
    combs = nchoosek(1:DOF, 2);
    Ncombs = size(combs, 1);
    for k = 1:Ncombs
        i = combs(k, 1); j = combs(k, 2);
        corJointMask = all( vertcat( Ym(3*DOF + i, :) == uint8(1), ...
                                     Ym(3*DOF + j, :) == uint8(1)), 1);
        h = min(i, j);
        trigMask = any( Ym(1:h, :) + Ym((1:h) + DOF, :) + Ym((1:h) + 2*DOF, :) > uint8(1), 1);
        removeMask = all( vertcat(corJointMask, trigMask), 1);
        mask(removeMask) = false;
    end
        
    Ym_trim = Ym(:, mask);

end

function Up_vel = parseUp_acc(Up_acc_in, DH_ext)

    if iscell(Up_acc_in)
        % Need to concatenate. Don't need to reduce since these
        % functions are already unique and will be reduced later
        % anyway.
        
        if coder.target('matlab')
            Up_acc = horzcat(Up_acc_in{:});
        else
            % Coder needs a bit more help with this command
            c = 0;
            for i = 1:numel(Up_acc_in)
                c = c+size(Up_acc_in{i}, 2);
            end
            Up_acc = zeros( size(Up_acc_in{1}, 1), c, class(Up_acc_in{1}));
            c = 0;
            for i = 1:numel(Up_acc_in)
                s = size(Up_acc_in{i}, 2);
                Up_acc(:, (c+1):(c+s)) = Up_acc_in{i};
                c = c + s;
            end
        end
    else
        Up_acc = Up_acc_in;
    end

    Up_vel = PSDM.getSetUpsilon_diff(Up_acc, DH_ext);
    
end