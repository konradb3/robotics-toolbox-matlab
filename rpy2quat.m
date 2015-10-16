function [ quat ] = rpy2quat( rpy )
%RPY2QUAT Summary of this function goes here
%   Detailed explanation goes here

phi = rpy(1) / 2.0;
the = rpy(2) / 2.0;
psi = rpy(3) / 2.0;

x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

quat = [w x y z];
end

