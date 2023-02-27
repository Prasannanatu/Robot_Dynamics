function twist_inB = adjoint(twist_inA,T_AB)

    p = T_AB(1:3,4);
    R = T_AB(1:3,1:3);
    skewM= skew(p);

    Ad_T = [R zeros(3,3); skewM*R R;];

    twist_inB = Ad_T * twist_inA;

    % twist_inB = ...
end

