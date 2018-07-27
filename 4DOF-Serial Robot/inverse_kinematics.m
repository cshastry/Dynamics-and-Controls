function [ s ] = inverse_kinematics( start, l1, l2, l0 )
%INIT_STATE Initialize 8 x 1 state vector
% input:
%   start  - 1x4, [x, y, z, yaw], initial gripper state vector;
%   l1     - length of link 1;
%   l2     - length of link 2;
%   l0     - initial height of end-effector;
% output:
%   s      - 4x1, [q1; q2; q3; q4], for
%            initial state, all joint velocity should be zero.

s(1) = atan2(start(1),start(2)); %Projection Method
s(3) = start(4); % q3 = yaw
sol = ikin(start,l1,l2,l0); 
% sol contains multiple solutions.Solution corresponding to elbow down and
% 1st octant is chosen
s = [s(1) sol(1,1) s(3) sol(1,2)].';
%% Check
if(imag(sum(s))~=0)
    error('Invalid configuration chosen, that configuration is probably outside of your reachable volume in current configuration');
end