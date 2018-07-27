function [ manipulator, gripper ] = manipulator_pos( state_pos, l1, l2, l0 )
%MANIPULATOR_POS coordinates of manipulator's position in world frame
% inputs:
%   state_pos: [q1; q2; q3; q4];
%   l1: length of link 1;
%   l2: length of link 2;
%   l0: initial height of end-effector;
% outputs:
%   manipulator: [x1 y1 z1; x2 y2 z2; x3 y3 z3]; joint locations
%   gripper: [x y z yaw]. Gripper coordinates.

% %% Forward Kinematics
% manipulator = [0 l1 0;
%                0 l1+l2 0;
%                0 l1+l2 -l0;];
% gripper = [0 l1+l2 -l0 0];

%Rotation Matrices
R_SA=[cos(state_pos(1)) -sin(state_pos(1)) 0; sin(state_pos(1)) cos(state_pos(1)) 0;0 0 1]; %Frame S to A
R_AB=[1 0 0;0 cos(state_pos(2)) -sin(state_pos(2));0 sin(state_pos(2)) cos(state_pos(2))]; % Frame A to B
R_BC=eye(3); %Frame B to C
R_CT=[cos(state_pos(3)) -sin(state_pos(3)) 0; sin(state_pos(3)) cos(state_pos(3)) 0;0 0 1]; %Frame C to T

%Homogenous Transforms from corresponding frames
H_SA=[R_SA [0 0 0].';0 0 0 1];
H_AB=[R_AB [0 l1 0].';0 0 0 1];
H_BC=[R_BC [0 l2 0].';0 0 0 1];
H_CT=[R_CT [0 0 state_pos(4)-l0].';0 0 0 1];

%Output of Joint Locations and Gripper state
manipulator=[subsref(H_SA*H_AB, struct('type', '()', 'subs', {{1:3,4}})),subsref(H_SA*H_AB*H_BC, struct('type', '()', 'subs', {{1:3,4}})),subsref(H_SA*H_AB*H_BC*H_CT, struct('type', '()', 'subs', {{1:3,4}}))].';
gripper=[subsref(H_SA*H_AB*H_BC*H_CT, struct('type', '()', 'subs', {{1:3,4}}));state_pos(3)].';

end

