function [ Rd,omegad ] = trajectory( t )
%TRAJECTORY 
%   Returns Desired Attitude and Desired Omega as a function of time

%% Sample Trajectory 1
if t<4
    
    Rd = eye(3);
    omegad = [0 0 0].';
    
else
    
    Rd = expm(225*pi/180*hat([0 0 1].'));
    omegad = [0 0 0].';
    
end
%% Sample Trajectory 2
% Rd = eye(3);
% omegad = [0 0 0].';
%     
% 
end

