function [newang, newrad] = resampleRadii(ang,rad,desiredSampleCount )
% resampleRadii resamples rad and their associated ang to a
% vector of length N and constant angular separation.
%ASSUMPTION, the value of ang always increases.
%ASSUMPTION, the input covers all 360deg (the first and last entry are
%joined together)
%This uses a similar technique to the simulated scanning function, hence
%the weird variable names

%convert to cartesian coords
map = cat(2,(cos(ang).*rad), (sin(ang).*rad));
map(length(map)+1,:)= map(1,:); %join first entry to the last
mapLines = zeros(length(map)-1,4); %preallocate for speed
for i =1:length(mapLines)
    mapLines(i,:) = [map(i,:) map(i+1,:)];  %converts into line format
end

%defines the resampling rate, you can play around with this if you don't want to
%sample all 360deg
startAngle =0;
endAngle = 2*pi;
newang= startAngle:abs(startAngle-endAngle)/desiredSampleCount:startAngle+endAngle- abs(startAngle-endAngle)/desiredSampleCount;
scanConfig =  cat(1,cos(newang), sin(newang))'
scanLines = cat(2,scanConfig*0, scanConfig)  %resample vectors

newrad = zeros(1,size(scanLines,1)); %preallocate
origin = repmat([0 0],length(mapLines),1); %preallocate
%casts rays to intersect the polygon generated from the input radii.  Finds
%the distance to the intersection point and set that as the new radius
for i =1:length(scanLines)
    crossingPoints = intersection(scanLines(i,:),mapLines);
    distanceSquared = sum((crossingPoints-origin).^2,2); %avoid sqrt on entire vector
    newrad(i) = sqrt(min(distanceSquared)); %only sqrt one value
end
end