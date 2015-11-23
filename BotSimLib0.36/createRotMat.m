function [ rotMat ] = createRotMat(angle)
%createRotMat Takes 2d vector and returns a rotation matrix with homogenous
%coords
rotMat =[cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0;0 0 1];
end