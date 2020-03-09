function tau = inverseDynamicsRegressorDensoAlt(Q, Qd, Qdd)

    [tau] = DensoAlt.stc_inverseDynamicsRegressor(Q, Qd, Qdd, []);
    
end

