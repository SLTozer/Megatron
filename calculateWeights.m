function [ weights, avgWeight, bestIndex ] = calculateWeights( particles, distances, sensorError )
% This function returns a vector of normalized real weights, the average un-normalized weight,
% and the index of the particle with highest weight. It accepts botSim[], a vector of distances,
% and a real standard deviation for the sensor error.

minDistance = min(distances);
count = length(particles);
totalWeight = 0;

bestWeight = 0;

weights(count,1) = 0;
% Calculate weights for each particle
% Weight is based solely on the distance to the nearest obstacle detected
% Uses the normal pdf with mean = minDistance and stdev = sensorError
for i = 1:count
    [partDists, ~] = particles(i).ultraScan();
    partMinDist = min(partDists);
    weights(i) = normpdf(partMinDist, minDistance, sensorError);
    if weights(i) > bestWeight
        bestWeight = weights(i);
        bestIndex = i;
    end
    totalWeight = totalWeight + weights(i);
end
% Set average (un-normalized) weight
avgWeight = totalWeight / count;
% Normalize weights
for i = 1:count
    weights(i) = weights(i) / totalWeight;
end

end

