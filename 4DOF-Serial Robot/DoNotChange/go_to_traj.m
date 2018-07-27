function [desired_state] = go_to_traj(t,goalPosition ,l1,l2,l0)
%GO_TO_TRAJ Summary of this function goes here
%   % trajhandle = @(t,l1,l2,l0)go_to_traj( t,[xTarget;yTarget;zTarget;yaw] ,l1,l2,l0)
  if t==0
      desired_state.pos = zeros(4,1);
  else
      desired_state.pos = inverse_kinematics(goalPosition,l1, l2, l0 );
  end
  desired_state.vel = zeros(4, 1);
  desired_state.acc = zeros(4, 1);
end

