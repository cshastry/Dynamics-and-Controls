function [W,delta] = noise( t )
%NOISE Summary of this function goes here
%   Detailed explanation goes here

fixed = [0.2 0.2 0.2].';

variable = 0.02*[sin(9*t) cos(9*t) 1/2*(sin(9*t) + cos(9*t))].';

delta = (fixed + variable); 
W = eye(3);

end

