function [ particles ] = resample( prevParticles, weights )
%This function returns botSim[], and accepts botSim[], a vector of real weights.

count = length(weights);
cumulativeWeights(count,1) = 0;
cumulativeWeights(1) = weights(1);
for i = 2:count
    cumulativeWeights(i) = cumulativeWeights(i-1) + weights(i);
end    

inverseCount = 1 / count;
stochSamp = rand() * inverseCount;
i = 1;

particles(count,1) = BotSim;

for j = 1:count
    while(stochSamp > cumulativeWeights(i))
        i = i + 1;
    end
    particles(j) = prevParticles(i);
    stochSamp = stochSamp + inverseCount;
end

end

