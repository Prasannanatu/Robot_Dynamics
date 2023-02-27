function T = twist2ht(S,theta)
% T = ...
    omega = S(1:3)';
    v = S(4:6);

    s_omega = [0 -omega(3) omega(2);
        omega(3) 0 -omega(1);
        -omega(2) omega(1) 0;];

    R = axisangle2rot(omega, theta);
    T = zeros (4);
    T(1:3, 1:3) = R;
    T(1:3,4) = (((theta*eye (3)) + ((1-cos (theta))* s_omega) + ((theta -sin (theta))*s_omega* s_omega))* v)'; 
    T(4,4) = 1;

end