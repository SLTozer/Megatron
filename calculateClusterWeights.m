function [weights,cWeights] = calculateClusterWeights(cCount,cWeights,weights,cIDs)

    for k = 1:length(weights)

        c = cIDs(k);
        weights(k) = weights(k)/cCount(c);

    end

    for l = 1:length(cWeights)

        cWeights(l) = cWeights(l)/cCount(l);

    end

end
