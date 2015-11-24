function [botSim] = localiseCluster(botSim,map,target)
% This function returns botSim, and accepts, botSim, a map and a target.
% LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

likelihoodField = LikelihoodField(modifiedMap,1,0.7,0.01);
routePlan = Map(modifiedMap, target, 5);
% Robot parameters
numScans = 8;
errorVal = [0, 0.4, 0.2];
num = getNumParticles(map, 6, 6, 0.9);

botSim.setScanConfig(botSim.generateScanConfig(numScans));
%generate some random particles inside the map
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap, errorVal);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setScanConfig(particles(i).generateScanConfig(numScans));
end

%% Localisation code
maxNumOfIterations = 50;
n = 0;
converged =0; %The filter has not converged yet

weightSlow = 0;
weightFast = 0;
slowDecay = 0.3;
fastDecay = 0.6;
sensorError = 1;
bestIndex = 0;
botScan = botSim.ultraScan(); %get a scan from the real robot.
particleScans = zeros(num,numScans,1);
for i = 1:num
    particleScans(i,:) = botScan;
end
tx = 0;
ty = 0;
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Apply particle weightings
    [weights, avgWeight, bestIndex] = calculateWeightsLF(particles, botScan, likelihoodField); % 36% of runtime
    % bestParticle = particles(bestIndex);
    weightSlow = weightSlow + (slowDecay * (avgWeight - weightSlow));
    weightFast = weightFast + (fastDecay * (avgWeight - weightFast));
    uncertainty = max([0 (1 - (weightFast/weightSlow))]);
    %% Resample and cluster particles
    %0.8s
    if n == 1
        [particles, clusterIDs, clusterCount] = ...
            clusterSample(particles, weights, 0, errorVal);
    else
        weights = normalizeClusterWeights(weights, clusterIDs, cCounts);
        [particles, clusterIDs, clusterCount] = ...
            clusterResample(particles, clusterIDs, clusterCount, weights, 0, errorVal);
    end
    % Set correct cluster data
    [clusterIDs, clusterCount] = removeEmptyClusters(clusterIDs);
    [cCounts, cWeights, cMeans, cBearings] = ...
        getClusterData( particles, weights, clusterIDs, clusterCount );
    % Merge clusters
    oldClusterCount = -1;
    while oldClusterCount ~= clusterCount
        oldClusterCount = length(cCounts);
        [clusterIDs, cCounts, cWeights, cMeans, cBearings] = ...
            mergeClusters(10, pi/4, clusterIDs, cCounts, cWeights, cMeans, cBearings);
        clusterCount = length(cCounts);
    end
    % 1.1s
    %% Write code to check for convergence
    estimatedDist = 0;
        n
    for i = 1:length(cMeans(:,1))
        i
        [cMeans(i,:), target]
        [norm(cMeans(i,:)-target), estimatedDist]
        if norm(cMeans(i,:)-target) > estimatedDist
            estimatedPos = cMeans(i,:);
            estimatedAng = cBearings(i,:);
            estimatedDist = norm(cMeans(i,:)-target);
        end
    end
    % estimatedPos = bestParticle.getBotPos();
    if (n == 1)
        tx = estimatedPos(1);
        ty = estimatedPos(2);
    end
    %% Write code to decide how to move next
    [xdiff, ydiff, bearing, distance] = routePlan.findBearing(estimatedPos, tx, ty);
    tx = xdiff + estimatedPos(1);
    ty = ydiff + estimatedPos(2);
    [tx, ty]
    deltaAng = bearing - estimatedAng;
    move = distance;
    if (distance > 10)
        move = 10;
    end
    botSim.turn(deltaAng);
    botSim.move(move);
    for i = 1:num
        particles(i).turn(deltaAng);
        particles(i).move(move);
    end
    % Reverse if we leave the map, because the simulation tends to mess up
    % pretty bad (irrecoverably) if the robot is scanning outside the map
    inside = botSim.insideMap();
    while ~inside
        botSim.move(-move);
        for i = 1:num
            particles(i).move(-move);
        end
        inside = botSim.insideMap();
    end
    % estimatedPos = bestParticle.getBotPos();
    targetDistance = norm([target(1) - estimatedPos(1), target(2) - estimatedPos(2)]);
    if (targetDistance < 3)
        converged = 1;
    end
    %end
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    %if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        for i =1:num
            particles(i).drawBot(3, 'y'); %draw particle with line length 3 and default color
        end
    botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    % bestParticle.drawBot(8, 'r');
    plot(estimatedPos(1),estimatedPos(2),'o');
    plot([estimatedPos(1) estimatedPos(1)+cos(estimatedAng)*15],...
        [estimatedPos(2) estimatedPos(2)+sin(estimatedAng)*15],'r');
    drawnow;
    %end
    %1.2s
    %pause(0.5);
end
end

%% Runtimes (sample of 1.2s per iteration on Map A, 523 particles)
% Calculating weights takes 36% of run time
% Ultrascanning takes 31% of run time
% Resampling takes 25% of run time
% Route Planning and movement takes 8% of run time
% The only real place for optimisation is in resampling/calculating weights
% and number of particles used
