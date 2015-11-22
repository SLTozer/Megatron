function [ particles, clusterIDs, clusters ] = clusterResample( prevParticles, prevClusterIDs, weights, uncertainty, errorVal )
% This function is similar to the resample function, except that it
% assigns clusters to the new particles equal to the cluster of their
% parents
% Currently DOES NOT make any use of the uncertainty value - TODO

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
clusterIDs = zeros(count);

for j = 1:count
    while(stochSamp > cumulativeWeights(i))
        i = i + 1;
    end
    particles(j) = BotSim(modMap, errorVal);
    particles(j).setBotPos(prevParticles(i).getBotPos());
    particles(j).setBotAng(prevParticles(i).getBotAng());
    clusterIDs(j) = prevClusterIDs(i);
    stochSamp = stochSamp + inverseCount;
end
end
