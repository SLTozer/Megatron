function [ weights ] = normalizeClusterWeights( weights, cIDs, cCounts )
% Normalizes the weights of all particles such that each cluster of
% particles has total weight proportional to its average weight, and the
% total weight of all particles is 1.

weightSum = 0;
% Calculate the weightsum and normalize particles by cluster size
for i = 1:length(weights)
    c = cIDs(i);
    weights(i) = weights(i) / cCounts(c);
    weightSum = weightSum + weights(i);
end
% Normalize particles to total weight 1
for i = 1:length(weights)
    weights(i) = weights(i) / weightSum;
end

end

