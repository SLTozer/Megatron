function [ particles, cIDs, clusterCount ] = clusterResample( ...
    prevParticles, prevCIDs, clusterCount, weights, uncertainty, errorVal )
% This function is similar to the resample function, except that it
% assigns clusters to the new particles equal to the cluster of their
% parents
% Random particles born of uncertainty will be assigned a new cluster

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

for j = 1:count
    while(stochSamp > cumulativeWeights(i))
        i = i + 1;
    end
    particles(j) = BotSim(modMap, errorVal);
    particles(j).setBotPos(prevParticles(i).getBotPos());
    particles(j).setBotAng(prevParticles(i).getBotAng());
    cIDs(j) = prevCIDs(i);
    if(uncertainty > rand())
        clusterCount = clusterCount + 1;
        particles(j).randomPose(0);
        cIDs(j) = clusterCount;
    end
    stochSamp = stochSamp + inverseCount;
end
end
