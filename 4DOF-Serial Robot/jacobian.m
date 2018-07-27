function [ J, Jdot ] = jacobian( q, v, l1, l2, l0 )
%JACOBIAN ( q, v, l1, l2, l0 )
% Find jacobian and its derivative for trajectory finding
% input:
%   q  - 4x1, [q1, q2, q3, q4]
%   v  - 4x1,  [dx,dy,dz,dyaw];
%   l1     - length of link 1;
%   l2     - length of link 2;
%   l0     - initial height of end-effector;
% output:
%   J     - 4x4 The jacobian matrix, depends on q
%   Jdot   -4x4 The  derivative of the jacobian matrix, depends on q and v

%J = eye(4);
%Jdot = eye(4);
[J,~] = jac(q,[0 0 0 0].',l1,l2,l0);
qdot = J\(v.');
[~,Jdot] = jac(q,qdot,l1,l2,l0);
end
