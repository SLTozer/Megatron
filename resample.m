function [ particles ] = resample( prevParticles, weights, uncertainty, errorVal )
% This function returns botSim[], and accepts botSim[], a vector of real
% weights, an uncertainty value in [0, 1], and an array of error values
% (as in BotSim).

count = length(weights);
cumulativeWeights(count,1) = 0;
cumulativeWeights(1) = weights(1);
for i = 2:count
    cumulativeWeights(i) = cumulativeWeights(i-1) + weights(i);
end    

inverseCount = 1 / count;
stochSamp = rand() * inverseCount;
i = 1;

modMap = prevParticles(1).getMap();

particles(count,1) = BotSim;

for j = 1:count
    while(stochSamp > cumulativeWeights(i))
        i = i + 1;
    end
    particles(j) = BotSim(modMap, errorVal);
    particles(j).setBotPos(prevParticles(i).getBotPos());
    particles(j).setBotAng(prevParticles(i).getBotAng());
    if(uncertainty > rand())
        particles(j).randomPose(0);
    end
    stochSamp = stochSamp + inverseCount;
end

end

