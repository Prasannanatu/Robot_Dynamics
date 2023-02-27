function J_v = jacobe(S,M,q)    
% Your code here
%     [r,c] = size(q);
%     I = eye(4);
%     for i=1:c
%         I = I * twist2ht(S(:,i), q(i));
%     end
% 
%      T= I * M;
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    R = [T(1,1:3);T(2,1:3);T(3,1:3)];
    P = T(1:3,4);
    
    skew_T = [0 -P(3) P(2);
              P(3) 0 -P(1);
              -P(2) P(1) 0];
    
    J_v = inv([R, [0 0 0;0 0 0;0 0 0];skew_T*R, R])*J_s;


end
