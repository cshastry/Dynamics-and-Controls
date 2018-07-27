function [ parameters ] = quad_params( )
%QUAD Summary of this function goes here
%   Detailed explanation goes here

field1 = 'inertia';
inertia = [5.5 0.06 -0.03;0.06 5.5 0.01;-0.03 0.01 0.1]*10^(-3);


field2 = 'r';
r = [1 0 0].';
r = r/norm(r);

field3 = 'v';
v1 = [0.174 -0.934 -0.034].';
v1 = v1/norm(v1);

v2 = [0 0.7071 0.7071].';
v2 = v2/norm(v2);

v3 = [-0.853 0.436 -0.286].';
v3 = v3/norm(v3);

v4 = [-0.122 -0.140 -0.983].';
v4 = v4/norm(v4);

v=[v1,v2,v3,v4];

field4 = 'theta';
theta1 = 40*pi/180;
theta2 = 40*pi/180;
theta3 = 40*pi/180;
theta4 = 20*pi/180;
theta = [theta1;theta2;theta3;theta4];

field5 = 'G';
G = diag([0.9 1.1 1.0]);

field6 = 'alpha';
alpha = 15;

field7 = 'kR';
kR = 0.4;

field8 = 'komega';
komega = 0.296;

field9 = 'kdelta';
kdelta = 0.5;

field10 = 'c';
c = 1.0;


parameters = struct(field1,inertia,field2,r,field3,v,field4,theta,field5,G,field6,alpha,field7,kR,field8,komega...
    ,field9,kdelta,field10,c);

end

