function [ acc ] = manipulatorDynamics( tau, pos, vel, params )
%MANIPULATORDYNAMICS ( tau, pos, vel, params ) manipulator dynamics model
%   tau     - 4 x 1, [tau_1; tau_2; tau_3; tau_4], torques or forces
%             generated from each joints;
%   pos     - 4 x 1, [q1; q2; q3; q4];
%   vel     - 4 x 1, [q1_vel; q2_vel; q3_vel; q4_vel];
%   params  - struc, output from manipulator() storing all robot 
%             parameters;
%   acc     - 4 x 1, [q1_acc; q2_acc; q3_acc; q4_acc];

parameters = [params.m1 params.m2 params.m3 params.m4 params.r1 params.r2 params.r3 params.r4 params.l1 params.l2 params.l3 params.l4 params.l0 params.grav].';

M = inertia(pos,parameters);
C = coriolis(pos,vel,parameters);
N = gravity(pos,vel,parameters);

% Equation of Motion is M*q_dotdot + C*qdot + N = Tau 
acc = M\(tau-C*vel-N);

end

