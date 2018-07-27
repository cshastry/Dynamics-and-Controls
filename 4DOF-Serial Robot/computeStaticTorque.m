function [ torques ] = computeStaticTorque( state_pos, F, l1, l2, l0 )
%COMPUTE_STATIC_TORQUE compute torques generated from motors to withstand
%given external load.
%   state_pos - [q1; q2; q3; q4];
%   l1        - length of link 1;
%   l2        - length of link 2;
%   l0        - initial height of end-effector;
%   F         - 4x1, [Fx; Fy; Fz; Mz], external load applied on gripper;
%   torques   - 4x1, [tau_1; tau_2; tau_3; tau_4], torque generated from
%               joint 1, joint 2, joint 3 and joint 4.

%%torques = zeros(4,1);
params = manipulator();

% Getting link and gripper jacobians
[J1,J2,J3,J_gripper] = link_jacs(state_pos,params.l1,params.l2,params.l0);

% Transforming forces on each link and gripper to corresponding Torques
T1 = J1.'*[0 0 -params.m1*params.grav 0].'
T2 = J2.'*[0 0 -params.m2*params.grav 0].'
T3 = J3.'*[0 0 -params.m3*params.grav 0].'
T4 = J_gripper.'*(F+[0 0 -params.m4*params.grav 0].');

% Torque Required to hold static position
torques = T1 + T2 + T3 + T4;

end

