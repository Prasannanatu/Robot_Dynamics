function J = jacob0(S,q)
% % Your code here
%     [rows_q cols_q] = size(q);
%     
%        
%     y = rand(6,rows_q);
% 
%     [rows cols] = size(S);
%     
%     for j = 1: cols
%         v = eye(4);
%         for i = 1:j
%             % Calculation of Transformation Matrix
%             v = v * twist2ht(S(:,i),q(i));
%         end
%         y(:,j) = adjoint(S(:,j),v);
%         
%     end
%     J = y;
    [row_q col_q] = size(q);
    [row col] = size(S);
    J_s = rand(6,col_q);

    
    
    for j = 1: col
        t_ = eye(4);
        for i = 1:j
            t_ = t_ * twist2ht(S(:,i),q(i));
        end
        J_s(:,j) = adjoint(S(:,j),t_);
        
    end
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    % T = twist2ht(V,omega);
    
    % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
    % adjoint(V,T)
    J = J_s;    
end