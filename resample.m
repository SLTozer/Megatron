function [ particles ] = resample( prevParticles, weights, uncertainty )
% This function returns botSim[], and accepts botSim[], a vector of real
% weights, and an uncertainty value in [0, 1].

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
    if(uncertainty > rand())
        particles(j).randomPose(0);
    end
    stochSamp = stochSamp + inverseCount;
end

end

