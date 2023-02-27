function T = fkine(S,M,q,frame)
    t_= eye(4);
   [row_s col_s] = size(S);
    
    for i = 1: col_s
        t_ = t_ * twist2ht(S(:,i),q(i));
    end
    
    if  strcmp(frame, 'body')==1
        T = M*t_;
    elseif strcmp(frame,'space')==1    
        T = t_*M;
    end
    % Your code here
end