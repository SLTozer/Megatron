function [ distance ] = disToLine(point,line )
%disToLine Returns the shortest distance between a point and an infinite line
a = line(:,[1 2]);
point = repmat(point,size(line,1),1);
vec = line(:,[3 4]) - line(:,[1 2]);
tm = repmat((sum(vec.^2,2)).^0.5,1,2);
n = vec ./ tm;
c = a-point;
dotva = repmat(dot(c,n,2),1,2).*n;
distance = (sum(c - dotva,2).^2).^0.5;
end

