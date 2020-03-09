function tau = inverseDynamicsDensoAlt(Q, Qd, Qdd)

    N = size(Q, 2);
    DOF = 6;
    tau = zeros(DOF, N);
        
    for i = 1:N
        [D, C, G] = DensoAlt.stc_EOM(Q(:, i), Qd(:, i));
    
        tau(:, i) = D*Qdd(:, i) + C*Qd(:, i) + G;
    end
    
end

