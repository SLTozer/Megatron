function [ weights ] = calculateWeights( particles, distances, sensorError )
%This function returns a vector of real weight, and accepts botSim[], a
%vector of distances, and a real standard deviation for the sensor error.

minDistance = min(distances);
count = length(particles);
totalWeight = 0;

weights(count,1) = 0;
% Calculate weights for each particle
% Weight is based solely on the distance to the nearest obstacle detected
% Uses the normal pdf with mean = minDistance and stdev = sensorError
for i = 1:count
    [partDists, ~] = particles(i).ultraScan();
    partMinDist = min(partDists);
    weights(i) = normalpdf(partMinDist, minDistance, sensorError);
    totalWeight = totalWeight + weights(i);
end
% Normalize weights
for i = 1:count
    weights(i) = weights(i) / totalWeight;
end

end

