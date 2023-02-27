function R = axisangle2rot(omega,theta)
    w_x = omega(1); 
    w_y = omega(2); 
    w_z = omega(3);

    skwsm = [0 -w_z w_y;w_z 0 -w_x;-w_y w_x 0];
    R = eye(3) + sin(theta)*skwsm + (1-cos(theta))*(skwsm^2);
    
% Your code here
end