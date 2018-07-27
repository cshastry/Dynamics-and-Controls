function  [tau,delta_cap_dot]  = controller( R,omega,Rd,omegad,W,delta_cap,parameters )
%CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

% if det(R)<0
%     norm(det(R))
%     det(R)
%     R
%     error('rotation')
% end 

A = (1/2)*trace(parameters.G*(eye(3) - Rd.'*R));                                                                % Attractive Potential
e_RA = (1/2)*inv_hat(parameters.G*Rd.'*R - R.'*Rd*parameters.G);                                                    % Attractive Error

% Generating repulsive potentials and errors corresponding to i constraints

B = zeros(length(parameters.theta),1);
e_RB = zeros(3,length(parameters.theta));

for i = 1:length(parameters.theta)
    
    % Repulsive Potential for each debris vector
    B(i) = 1 - (1/parameters.alpha)*log((cos(parameters.theta(i)) - (parameters.r).'*R.'*(parameters.v(:,i)))/(1 + cos(parameters.theta(i)))); 
    
    % Repulsive error related to each debris vector
    e_RB(:,i) = hat(R.'*(parameters.v(:,i)))*parameters.r/(parameters.alpha*(parameters.r.'*R.'*(parameters.v(:,i)) - cos(parameters.theta(i))));

end

e_R = e_RA * sum(B) + e_RB * (A * ones(length(parameters.theta),1));                                  % Rotation error vector
e_omega = omega - R.'*Rd*omegad;                                                                    % Angular Velocity error vector

%Parameter Estimation


tau = -parameters.kR*e_R - parameters.komega*e_omega + cross(omega,parameters.inertia*omega) - W*delta_cap;   % Input Moments

delta_cap_dot = parameters.kdelta*W.'*(e_omega + parameters.c*e_R);


end

