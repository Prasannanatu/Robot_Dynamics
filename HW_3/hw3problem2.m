% RBE 501 - Robot Dynamics - Fall 2022
% Homework 3, Problem 2
% Worcester Polytechnic Institute
%
% Creator: L. Fichera <lfichera@wpi.edu>
% Modified: A. Rosendo <arosendo@wpi.edu>
clear, clc, close all
addpath('utils');

plotOn = true; 
nTests = 10;

%% Create the manipulator
mdl_stanford
stanf

if plotOn
   stanf.teach(zeros(1,6)); 
end

%% YOUR CODE HERE
%% Part A - Forward Kinematics via PoE in the body frame
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

% Joint limits
q_link = stanf.qlim; 



S_body = [0 0 1 0 -0.154 0;
         -1 0 0 0 0.263 0;
          0 0 0 0 0 1;
          0 0 1 0 0 0;
          0 1 0 0.263 0 0;
          0 0 1 0 0 0]';




S_space = [0 0 1 0 0 0;
           0 1 0 -0.412 0 0;
           0 0 0 0 0 1;
           0 0 1 0.154 0 0;
           1 0 0 0 0.412 -0.154;
           0 0 1 0.154 0 0]';


% Let us calculate the homogeneous transformation matrix M for the
% home configuration
R_home = [0 -1 0; 1 0 0; 0 0 1]';
t_home = [0 0.154 0.675]';
M = [R_home t_home; 0 0 0 1];
q = zeros(1,6);
% Joint limits
q_link = stanf.qlim; 
fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [q_link(1,1) + (q_link(1,2) - q_link(1,1)) * rand(), ...
        q_link(2,1) + (q_link(2,2) - q_link(2,1)) * rand(), ...
        q_link(3,1) + (q_link(3,2) - q_link(3,1)) * rand(), ...
        q_link(4,1) + (q_link(4,2) - q_link(4,1)) * rand(), ...
        q_link(5,1) + (q_link(5,2) - q_link(5,1)) * rand(), ...
        q_link(6,1) + (q_link(6,2) - q_link(6,1)) * rand()];
    
    % Calculate the forward kinematics
    T = fkine(S_body,M,q,'body');
    
    if plotOn
        stanf.teach(q);
        title('Forward Kinematics Test');
    end
    
%     assert(all(all(abs(double(stanf.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [q_link(1,1) + (q_link(1,2) - q_link(1,1)) * rand(), ...
        q_link(2,1) + (q_link(2,2) - q_link(2,1)) * rand(), ...
        q_link(3,1) + (q_link(3,2) - q_link(3,1)) * rand(), ...
        q_link(4,1) + (q_link(4,2) - q_link(4,1)) * rand(), ...
        q_link(5,1) + (q_link(5,2) - q_link(5,1)) * rand(), ...
        q_link(6,1) + (q_link(6,2) - q_link(6,1)) * rand()];
    
    % Calculate the Jacobian in the body frame
    J_b = jacobe(S_body,M,q);
    
    if plotOn
        stanf.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)];% swap the rotation and translation components
%     assert(all(all(abs(double(stanf.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


% Part C - Calculate the Analyical Jacobian of the manipulator
fprintf('---------------------Analytical Jacobian Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Analytical Jacobian for 10 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [q_link(1,1) + (q_link(1,2) - q_link(1,1)) * rand(), ...
        q_link(2,1) + (q_link(2,2) - q_link(2,1)) * rand(), ...
        q_link(3,1) + (q_link(3,2) - q_link(3,1)) * rand(), ...
        q_link(4,1) + (q_link(4,2) - q_link(4,1)) * rand(), ...
        q_link(5,1) + (q_link(5,2) - q_link(5,1)) * rand(), ...
        q_link(6,1) + (q_link(6,2) - q_link(6,1)) * rand()];
    
    % Calculate the Analytical Jacobian
    J_a = jacoba(S_space,M,q);
    
    if plotOn
        stanf.teach(q);
        title('Analytical Jacobian Test');
    end
    
    % Test the correctness of the Jacobian
    Jref = stanf.jacob0(q);
    Jref = Jref(1:3,:);
%     assert(all(all(abs(double(Jref) - J_a) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = zeros(1,6);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.5 * cos(t);
y = 0.5 * sin(t);
z = 0.7 * ones(1,nTests);
path = [x; y; z];

if plotOn
    stanf.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S_body, M, currentQ, 'body');

    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        % YOUR INVERSE KINEMATICS CODE HERE 
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
        J_a = jacoba(S_space,M,currentQ);

        Delta_Q = pinv(jacoba(S_space,M,currentQ)) * (targetPose-currentPose);

        currentQ = currentQ + Delta_Q';

        T = fkine(S_body, M, currentQ, 'body');

        currentPose = T(1:3,4);
        
        if plotOn
            try
                stanf.teach(currentQ);
                plot_ellipse(J_a*J_a', currentPose,'alter',h);
            catch ME
                continue
            end
        end
    end
end

fprintf('\nTest passed successfully.\n');