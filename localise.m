function [botSim] = localise(botSim,map,target)
% This function returns botSim, and accepts, botSim, a map and a target.
% LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

routePlan = Map(modifiedMap, target, 0);
% Robot parameters
numScans = 8;
errorVal = [0, 0.2, 0.1];
num =1000; % number of particles

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
slowDecay = 0.2;
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
    
    %% Write code for updating your particles scans
    for i = 1:num
        particles(i).setScanConfig(particles(i).generateScanConfig(numScans));
        particleScans(i,:) = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles    
    [weights, avgWeight, bestIndex] = calculateWeights(particles, botScan, sensorError);
    weightSlow = weightSlow + (slowDecay * (avgWeight - weightSlow));
    weightFast = weightFast + (fastDecay * (avgWeight - weightFast));
    uncertainty = max([0 (1 - (weightFast/weightSlow))]);
    %% Write code for resampling your particles
    particles = resample(particles, weights, uncertainty, errorVal);
    
    %% Write code to check for convergence    
    estimatedPos = particles(bestIndex).getBotPos();
    targetDistance = norm([target(1) - estimatedPos(1), target(2) - estimatedPos(2)]);
    if (n == 1)
        tx = estimatedPos(1);
        ty = estimatedPos(2);
    end
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    %if (n < 5)
    %    turn = 0.5;
    %    move = 2;
    %    botSim.turn(turn); %turn the real robot.  
    %    botSim.move(move); %move the real robot. These movements are recorded for marking 
    %    for i =1:num %for all the particles. 
    %        particles(i).turn(turn); %turn the particle in the same way as the real robot
    %        particles(i).move(move); %move the particle in the same way as the real robot
    %    end
    %else
    [xdiff, ydiff, bearing, distance] = routePlan.findBearing(estimatedPos, tx, ty);
    tx = xdiff + estimatedPos(1);
    ty = ydiff + estimatedPos(2);
    [tx, ty]
    direction = particles(bestIndex).getBotAng();
    deltaAng = bearing - direction;
    move = distance;
    if (distance > 3 || targetDistance > 3)
        move = 3;
    else
        move = targetDistance;
    end
    botSim.turn(deltaAng);
    botSim.move(move);
    for i = 1:num
        particles(i).turn(deltaAng);
        particles(i).move(move);
    end
    estimatedPos = particles(bestIndex).getBotPos();
    targetDistance = norm([target(1) - estimatedPos(1), target(2) - estimatedPos(2)]);
    if (targetDistance < 3)
        converged = 1;
    end
    %end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    %if botSim.debug()
    %    hold off; %the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    particles(bestIndex).drawBot(3, 'b');
    %    for i =1:num
    %        particles(i).drawBot(3); %draw particle with line length 3 and default color
    %    end
    drawnow;
    %end
    %pause(0.5);
end
end
