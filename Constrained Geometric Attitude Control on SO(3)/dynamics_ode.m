function state_dot = dynamics_ode( t,state,controller_handle,trajectory_handle,disturbance_handle,parameters )
%DYNAMICS_ODE Summary of this function goes here
%   Detailed explanation goes here

R = reshape(state(1:9),3,3);

%% Projecting onto rotation matrix space
% [U,~,V] = svd(R);
% R = U*V.';
omega = state(10:12);

%% Disturbance torque
[W,delta] = disturbance_handle(t);
disturbance = W*delta;

%% Desired trajectory generation
[R_des,omega_des] = trajectory_handle(t);

% Projecting onto rotation matrix space
% [U,~,V] = svd(R_des);
% R_des = U*V.';

%% Generating control torques based on desired state
[controller_torque,delta_cap_dot] = controller_handle(R,omega,R_des,omega_des,W,state(13:15),parameters);

% Net external torque on system
external_torque = controller_torque + disturbance;

% Dynamics
[R_dot,omega_dot] = quadrotorDynamics(external_torque,R,omega,parameters);

state_dot = [R_dot(:);omega_dot;delta_cap_dot];

end

