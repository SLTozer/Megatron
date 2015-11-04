function testRealbot(robot,map,sensorError)
% This function accepts a Robot, and a map.


%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself


%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    % SET ACCORDING TO REAL ROBOT'S SCANNING
    particles(i).setScanConfig(particles(i).generateScanConfig(20));
end

%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet

weightSlow = 0;
weightFast = 0;
slowDecay = 0.2;
fastDecay = 0.6;
bestIndex = 0;
botScan = botSim.ultraScan(); %get a scan from the real robot.
particleScans = zeros(num,20,1);
for i = 1:num
    particleScans(i,:) = botScan;
end
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    % GET ROBOT SCANS
    
    %% Write code for updating your particles scans
    for i = 1:num
        particleScans(i,:) = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles    
    [weights, avgWeight, bestIndex] = calculateWeights(particles, botScan, sensorError);
    weightSlow = weightSlow + (slowDecay * (avgWeight - weightSlow));
    weightFast = weightFast + (fastDecay * (avgWeight - weightFast));
    uncertainty = max([0 (1 - (weightFast/weightSlow))]);
    %% Write code for resampling your particles
    particles = resample(particles, weights, uncertainty);
    
    %% Write code to check for convergence    
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    robot.turn(turn); %turn the real robot.  
    robot.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        particles(1).drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
