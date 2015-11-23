function [ cCount, cWeight, cMean, cDev ] = getClusterData( particles, weights, clusterIDs, clusterCount )
% cCount  = The number of particles in the cluster
% cMean   = The mean of the cluster position
% cDev    =  The standard deviation of the cluster position
% cWeight = The avg weight of particles in the cluster

cCount = zeros([clusterCount,1]);
cMean = zeros([clusterCount,2]);
cDev = zeros([clusterCount,2]);
cWeight = zeros([clusterCount,1]);

% Sum counts, positions, and weights for each cluster
for i = 1:length(particles)
    c = clusterIDs(i);
    cCount(c) = cCount(c) + 1;
    cMean(c,:) = cMean(c,:) + particles(i).getBotPos();
    cWeight(c) = cWeight(c) + weights(i);
end
% Divide positions and weights by count to receive the cluster average
for i = 1:clusterCount
    cMean(i,:) = cMean(i,:) / cCount(i);
    cWeight(i) = cWeight(i) / cCount(i);
end

% Use the mean to calculate the cluster standard deviation
for i = 1:length(particles)
    c = clusterIDs(i);
    cDev(c,:) = cDev(c,:) + ((particles(i).getBotPos() - cMean(c,:)).^2);
end
for i = 1:clusterCount
    cDev(i,:) = sqrt(cDev(i,:) / cCount(i));
end

end

