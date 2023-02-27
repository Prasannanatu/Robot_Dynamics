function Vb = adjoint(Va,T)
%     p = T(1:3,4);
%     R = T(1:3,1:3);
%     skewM= skew(p);
% 
%     Ad_T = [R zeros(3,3); skewM*R R;];
% 
%     Vb = Ad_T * Va;
%     V = Va;
    R = [T(1,1:3);T(2,1:3);T(3,1:3)];
    P = T(1:3,4);
    Vb = [Va(4) Va(5) Va(6)];
    
    
    skew_T = [0 -P(3) P(2);
              P(3) 0 -P(1);
              -P(2) P(1) 0];
    
    Vb = [R, [0 0 0;0 0 0;0 0 0];skew_T*R, R] * Va;
    %skew_V = [skew_omg, [V(4) V(5) V(6)]'; 0 0 0 0];

    % Your code here
end

