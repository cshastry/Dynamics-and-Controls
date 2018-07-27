function [ tau ] = controller( qd, t, params )
%CONTROLLER manipulator controller
%   qd     - the current states: qd.pos, qd.vel;
%            the desired states: qd.pos_des, qd.vel_des, qd.acc_des;
%   t      - time, unit is second;
%   params - output from manipulator() storing all robot parameters;
%   tau    - 4 x 1, controller output.
% Compute the desired controls

tau = zeros(4,1);

%% Gain Matrices

Kp = 32*eye(4);
Kd = 2*eye(4);

%% Computing Controller Torque

parameters = [params.m1 params.m2 params.m3 params.m4 params.r1 params.r2 params.r3 params.r4 params.l1 params.l2 params.l3 params.l4 params.l0 params.grav].';

% Defining Errors
e_pos = -(qd.pos_des - qd.pos);
e_vel = -(qd.vel_des - qd.vel); 

% Generating M , C and N matrices
M = inertia(qd.pos,parameters);
C = coriolis(qd.pos,qd.vel,parameters);
N = gravity(qd.pos,qd.vel,parameters);

% Computed Torques (Control Law)
tau = M*(qd.acc_des-Kd*e_vel-Kp*e_pos) + C*qd.vel + N;

end

