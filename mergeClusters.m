function [ cIDs, clusterCount ] = mergeClusters( cIDs, cMeans, cDev, clusterCount )
% Takes the set of clusters and merges pairs of clusters that overlap
% A pair of clusters overlap when each pair's mean is within 3 standard
% deviations of the other

i = 1;
while i <= clusterCount-1
    j = i + 1;
    while j <= clusterCount
        % Check distance using the smaller standard deviation
        minDev = min([norm(cDev(i,:)), norm(cDev(j,:))]);
        meanDist = norm(cMeans(i,:)-cMeans(j,:));
        % Merge clusters if they are within 3 standard deviations
        if meanDist < (minDev * 3)
            for p = 1:length(cIDs)
                if cIDs(p) == j
                    cIDs(p) = i;
                elseif cIDs(p) > j
                    cIDs(p) = cIDs(p) - 1;
                end
            end
            clusterCount = clusterCount - 1;
            cDev = cDev([1:j-1,j+1:end],:);
            cMeans = cMeans([1:j-1,j+1:end],:);
        else
            j = j + 1;
        end
    end
    i = i + 1;
end

end

