function [ weights, avgWeight, bestIndex ] = calculateWeights( particles, distances, sensorError )
% This function returns a vector of normalized real weights, the average un-normalized weight,
% and the index of the particle with highest weight. It accepts botSim[], a vector of distances,
% and a real standard deviation for the sensor error.

count = length(particles);
totalWeight = 0;

bestWeight = 0;
bestIndex = 1;

weights(count,1) = 0;
% Calculate weights for each particle
% Weight is based solely on the distance to the nearest obstacle detected
% Uses the normal pdf with mean = minDistance and stdev = sensorError
for i = 1:count
    [partDists, ~] = particles(i).ultraScan();
    % Calculate weight here
    weights(i) = 1;
    for j = 1:length(distances)
        tmp = normpdf(partDists(j), distances(j), sensorError*5);
        weights(i) = weights(i) * tmp;
    end
    %
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

