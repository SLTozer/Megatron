function testRealbot(robot,map,target)
    % This function returns botSim, and accepts, botSim, a map and a target.
    % LOCALISE Template localisation function

    %% setup code
    %you can modify the map to take account of your robots configuration space
    modifiedMap = map; %you need to do this modification yourself
    %botSim.setMap(modifiedMap);

    likelihoodField = LikelihoodField(modifiedMap,1,0.8,0.005);
    routePlan = Map(modifiedMap, target, 10); % NB due to standard map being defined counterclockwise, shrinking in opposite direction
    % Robot parameters
    numScans = robot.scan_num;
    errorVal = [1, 0.1, 0.1]; % scan std dev, movement cm std dev per cm, radians std dev per rad
    num = 1000; % number of particles

    robot.scan_num = numScans;
    %botSim.setScanConfig(botSim.generateScanConfig(numScans));
    %generate some random particles inside the map
    particles(num,1) = BotSim; %how to set up a vector of objects
    for i = 1:num
        particles(i) = BotSim(modifiedMap, errorVal);  %each particle should use the same map as the botSim object
        particles(i).randomPose(10); %spawn the particles in random locations
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
    botScan = robot.ultraScan(); %get a scan from the real robot.
    particleScans = zeros(num,numScans,1);
    for i = 1:num
        particleScans(i,:) = botScan;
    end
    tx = 0;
    ty = 0;
    
    while(converged == 0 && n < maxNumOfIterations) %%particle filter loop        
        n = n+1; %increment the current number of iterations
        botScan = robot.ultraScan() %get a scan from the real robot.

        %% Write code for updating your particles scans
        for i = 1:num
            particles(i).setScanConfig(particles(i).generateScanConfig(numScans));
            particleScans(i,:) = particles(i).ultraScan();
        end

        %% Write code for scoring your particles    
        %[weights, avgWeight, bestIndex] = calculateWeights(particles, botScan, sensorError);
        [weights, avgWeight, bestIndex] = calculateWeightsLF(particles, botScan, likelihoodField);
        weightSlow = weightSlow + (slowDecay * (avgWeight - weightSlow));
        weightFast = weightFast + (fastDecay * (avgWeight - weightFast));
        uncertainty = max([0 (1 - (weightFast/weightSlow))]);
        
        %% Write code to check for convergence   
        bestParticle = particles(bestIndex);
        estimatedPos = bestParticle.getBotPos();
        targetDistance = norm([target(1) - estimatedPos(1), target(2) - estimatedPos(2)]);
        if (targetDistance < 3)
            converged = 1;
            estimatedPos
        end
        if (n == 1)
            tx = estimatedPos(1);
            ty = estimatedPos(2);
        end
        %% Write code for resampling your particles
        particles = resample(particles, weights, uncertainty, errorVal);

        
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
        direction = bestParticle.getBotAng();
        deltaAng = wrapToPi(bearing - direction)
        if (n < 5 && distance > 4)
            move = 4;
        elseif (distance > 10)
            move = distance / 2;
        else
            move = distance;
        end
        robot.turn(deltaAng);
        robot.move(move);
        for i = 1:num
            particles(i).turn(deltaAng);
            particles(i).move(move);
        end
        bestParticle.turn(deltaAng);
        bestParticle.move(move);
       

        %% Drawing
        %only draw if you are in debug mode or it will be slow during marking
        %if botSim.debug()
        figure
        hold on
        bestParticle.drawMap();
        routePlan.plot(estimatedPos);
        %Robot.plotScan(estimatedPos, direction, botScan);
        for i =1:num
            particles(i).drawBot(3,'b'); %draw particle with line length 3 and default color
        end
        bestParticle.drawBot(15, 'g');
        drawnow;
        %end
        %pause(0.5);
    end
end
