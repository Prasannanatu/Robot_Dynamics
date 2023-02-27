function T = twist2ht(S,theta)
% Your code here
    omega = [S(1) S(2) S(3)]; % angular vecolity     
     vel = [S(4) S(5) S(6)]; % linear velocity
     
     skwm = [0 -S(3) S(2);S(3) 0 -S(1);-S(2) S(1) 0];
     
     % If needed, you can calculate a rotation matrix with:     
     R = axisangle2rot(omega,theta);         
     
     T = [R , (eye(3)*theta + (1-cos(theta))* skwm + (theta - sin(theta))*(skwm^2)) * vel' ;0 0 0 1];
end