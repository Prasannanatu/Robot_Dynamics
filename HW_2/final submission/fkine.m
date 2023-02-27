function T = fkine(S,M,q)
    %T = ...
    [r,c] = size(q);
    I = eye(4);
    for i=1:c
        I = I * twist2ht(S(:,i), q(i));
    end

     T= I * M;

end