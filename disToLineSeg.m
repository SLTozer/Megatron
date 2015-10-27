function [ distances ] = disToLineSeg(point,line )
%disToLineSeg Returns the shortest distance between a point and line segment
lStart = line(:,[1 2]);
lEnd = line(:,[3 4]);
point = repmat(point,size(line,1),1);
vec = lEnd - lStart;
lenSQ = sum(vec.^2,2);
t = dot(point-line(:,[1 2]),vec,2)./lenSQ;
distances = zeros(length(t),1);
for i=1:length(t)
    if t(i) <0
        distances(i) = distance(point(i,:),lStart(i,:));
    elseif t(i) >1
        distances(i) = distance(point(i,:),lEnd(i,:));
    else
        projection = lStart(i,:) + t(i) * (vec(i,:));
        distances(i) = distance(point(i,:), projection);
    end
end
end

% line = [-2 0 0 -2; 3 -1 3 10; 3 1 11 1;0 0 0 0]
% point = [0 0];