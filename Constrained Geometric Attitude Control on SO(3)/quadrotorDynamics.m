function [R_dot,omega_dot] = quadrotorDynamics( tau_ext,R,omega,parameters )
%QUADROTORDYNAMICS Summary of this function goes here

% INPUTS : tau_ext = external torque into system (sum of controller torque and Disturbance torque)
%          R = Current attitude
%          omega = Current angular velocity
%          parameters = System Parameters
% OUTPUTS : R_dot : Derivative of current rotation matrix
%           omega_dot : Angular Acceleration 

% Evolution of rotation matrix
R_dot = R * hat(omega);

% Attitude Dynamics
omega_dot = parameters.inertia\(tau_ext - cross(omega,parameters.inertia*omega));

end

