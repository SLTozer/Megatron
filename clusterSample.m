function [ particles, cIDs, clusterCount ] = clusterSample( ...
    prevParticles, weights, uncertainty, errorVal )
% This function is similar to the resample function, except that it
% additionally generates a set of clusters - 1 for each particle that is
% resampled.

count = length(weights);
cumulativeWeights(count,1) = 0;
cumulativeWeights(1) = weights(1);
for i = 2:count
    cumulativeWeights(i) = cumulativeWeights(i-1) + weights(i);
end    

inverseCount = 1 / count;
stochSamp = rand() * inverseCount;
i = 1;

modMap = prevParticles(1).getMap();

particles(count,1) = BotSim;
cIDs = zeros([count,1]);

clusterCount = 0;
currentlyClustering = 0;

for j = 1:count
    while(stochSamp > cumulativeWeights(i))
        i = i + 1;
        currentlyClustering = 0;
    end
    % If i changed at the start of this iteration, create a new cluster
    if currentlyClustering == 0
        clusterCount = clusterCount + 1;
        currentlyClustering = 1;
    end
    particles(j) = BotSim(modMap, errorVal);
    particles(j).setBotPos(prevParticles(i).getBotPos());
    particles(j).setBotAng(prevParticles(i).getBotAng());
    cIDs(j) = clusterCount;
    if(uncertainty > rand())
        particles(j).randomPose(0);
    end
    stochSamp = stochSamp + inverseCount;
end
end

