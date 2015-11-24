function [ cCount, cWeight, cMean, cBearing ] = getClusterData( particles, weights, clusterIDs, clusterCount )
% cCount  = The number of particles in the cluster
% cMean   = The mean of the cluster position
% cDev    =  The standard deviation of the cluster position
% cWeight = The avg weight of particles in the cluster

cCount = zeros([clusterCount,1]);
cMean = zeros([clusterCount,2]);
cBearing = zeros([clusterCount,1]);
cWeight = zeros([clusterCount,1]);

vecSum = zeros([clusterCount,2]);

% Sum counts, positions, and weights for each cluster
for i = 1:length(particles)
    c = clusterIDs(i);
    cCount(c) = cCount(c) + 1;
    cMean(c,:) = cMean(c,:) + particles(i).getBotPos();
    cWeight(c) = cWeight(c) + weights(i);
    botAng = particles(i).getBotAng();
    vecSum(c,:) = vecSum(c,:) + [cos(botAng) sin(botAng)];
end
% Use the cluster count to find the cluster averages
for i = 1:clusterCount
    if(cCount(i) == 0)
        disp('Zero')
        i
        cMean(i,:)
        clusterCount
        % clusterIDs
        error('Zero');
    end
    cMean(i,:) = cMean(i,:) / cCount(i);
    cWeight(i) = cWeight(i) / cCount(i);
    vecSum;
    cBearing(i) = vectorAngle(vecSum(i,:));
end

end

