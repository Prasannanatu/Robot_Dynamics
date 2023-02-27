function J_a = jacoba(S,M,q)    
% Your code here
    J_s = jacob0(S,q);
    J_omega = J_s(1:3,:);
    J_vel= J_s(4:6,:);
    
    T = fkine(S,M,q,'space');
    P = T(1:3,4);
    
    J_a = J_vel - skew(P)*J_omega;
end