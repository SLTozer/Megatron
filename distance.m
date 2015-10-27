function [ distanceOut ] = distance( p1,p2 )
%distance returns the distance between two points
distanceOut =sqrt(sum((p1-p2).^2,2));
end