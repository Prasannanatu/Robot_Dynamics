function J = jacob0(S,q) 
   
     [rows_q cols_q] = size(q);
    
       
    y = rand(6,rows_q);

    [rows cols] = size(S);
    
    for j = 1: cols
        v = eye(4);
        for i = 1:j
            % Calculation of Transformation Matrix
            v = v * twist2ht(S(:,i),q(i));
        end
        y(:,j) = adjoint(S(:,j),v);
        
    end
    J = y;

end