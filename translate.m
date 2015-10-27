function [ output ] = translate(pos,transMat )
%translate takes a homogenous translatation matrix and 2d position vector
%and performs a translation on it.
posHomo = cat(2,pos,ones(size(pos,1),1));
output = (transMat*posHomo')';
output(:,3) = [];
end

