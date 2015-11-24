function [ clusterIDs, clusterCount ] = removeEmptyClusters( clusterIDs )
% Takes the list of cluster IDs and shifts their values such that the only
% IDs used are a contiguous block [1..clusterCount]

clusters = unique(clusterIDs);

clusterCount = length(clusters);
for i = 1:length(clusterIDs)
    for c = 1:length(clusters)
        if clusterIDs(i) == clusters(c)
            clusterIDs(i) = c;
            break;
        end
    end
end

end
