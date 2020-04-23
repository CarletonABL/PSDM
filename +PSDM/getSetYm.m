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
            Ym = Ycor;
        case 'centrifugal'
            Ym = Ycent;
        case 'joint'
            Ym = Yjoint;
        case 'velocity'
            Ym = horzcat(Ycor, Ycent);
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