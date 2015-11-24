function [ cIDs, cCount, cWeight, cMean, cBearing ] = mergeClusters( ...
    mergeDist, mergeAng, cIDs, cCount, cWeight, cMean, cBearing )
% Takes the set of clusters and merges pairs of clusters that overlap
% A pair of clusters overlap when each pair's mean is within 3 standard
% deviations of the other
% Recalculates the cluster data, but may not always merge all possible
% clusters after a single iteration - call repeatedly until no more
% clusters have been merged.

clusterCount = length(cCount);

i = 1;
while i <= clusterCount-1
    j = i + 1;
    while j <= clusterCount
        % Check distance using the smaller standard deviation
        angDiff = abs(cBearing(i) - cBearing(j));
        meanDist = norm(cMean(i,:)-cMean(j,:));
        % Merge clusters if they are within 3 standard deviations
        if meanDist <= mergeDist && angDiff < mergeAng
            % Assign new clusters
            for p = 1:length(cIDs)
                if cIDs(p) == j
                    cIDs(p) = i;
                elseif cIDs(p) > j
                    cIDs(p) = cIDs(p) - 1;
                end
            end
            % Merge positions
            cMean(i) = ((cMean(i) * cCount(i)) + (cMean(j) * cCount(j))) / (cCount(i) + cCount(j));
            % Merge weights
            cWeight(i) = ((cWeight(i) * cCount(i)) + (cWeight(j) * cCount(j))) / (cCount(i) + cCount(j));
            % Merge angles
            vecSum = (cCount(i)*[cos(cBearing(i)) sin(cBearing(i))])...
                + (cCount(j)*[cos(cBearing(j)) sin(cBearing(j))]);
            cBearing(i) = vectorAngle(vecSum);
            % Merge counts
            cCount(i) = cCount(i) + cCount(j);
            % Clear entries for the consumed cluster
            cCount = cCount([1:j-1, j+1:end]);
            cMean = cMean([1:j-1, j+1:end],:);
            cBearing = cBearing([1:j-1, j+1:end]);
            cWeight = cWeight([1:j-1, j+1:end]);
            clusterCount = clusterCount - 1;
        else
            j = j + 1;
        end
    end
    i = i + 1;
end

end

