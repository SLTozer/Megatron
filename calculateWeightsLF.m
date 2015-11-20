function [ weights, avgWeight, bestIndex ] = calculateWeightsLF( particles, distances, lf )
% This function returns a vector of normalized real weights, the average un-normalized weight,
% and the index of the particle with highest weight. It accepts botSim[], a vector of distances,
% and a likelihood field of the map.

    count = length(particles);
    totalWeight = 0;

    bestWeight = 0;
    bestIndex = 1;

    anglePerScan = 2*pi/length(distances);

    weights(count,1) = 0;
    % Calculate weights for each particle
    for i = 1:count
        % Calculate weight here
        weights(i) = 1;
        for j = 1:length(distances)
            if isnan(distances(j))
                weight = lf.minLikelihood;
            else
                scanAngle = particles(i).getBotAng() + ((j-1) * anglePerScan);
                scanVector = distances(j) * [cos(scanAngle) sin(scanAngle)];
                projectedPoint = particles(i).getBotPos() + scanVector;
                weight = lf.getPointLikelihood(projectedPoint);
            end
            weight = weight^scanWeight;
            weights(i) = weights(i) * weight;
        end
        if weights(i) > bestWeight
            bestWeight = weights(i);
            bestIndex = i;
        end
        totalWeight = totalWeight + weights(i);
        if isnan(weights(i))
            fprintf('Weight %d is NaN', i)
            distances
            partDists
        end
    end
    % Set average (un-normalized) weight
    avgWeight = totalWeight / count;
    if isnan(avgWeight)
        disp('Average weight is NaN')
        [totalWeight, count]
    end
    % Normalize weights
    for i = 1:count
        weights(i) = weights(i) / totalWeight;
    end

end

