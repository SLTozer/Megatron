function [ transMat ] = createTransMat( translation )
%translate Takes 2d vector and returns a translation matrix with homogenous
%coords
transMat =[1 0 translation(1);0 1 translation(2);0 0 1];
end