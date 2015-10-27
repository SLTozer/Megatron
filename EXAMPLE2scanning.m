clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

botSim.setBotPos([20 40]);
botSim.setBotAng(-pi/2);

botSim.drawMap();
botSim.drawBot(3); % draws robot with direction indicator with a length 3
%the length of the directin indicator does not matter, it just makes it
%easy to see where the robot is pointing
disp('This example shows how to take simulated ultrasound scans.')
disp('the function ultraScan() returns two arrays, one with the distances');
disp('and one with the crossing points of the scan rays.');
disp('By default the sensor is set up to scan in 6 places');
input('Press enter to scan:');
disp('-------------------------------------------------------------------');
disp(' ');
[distance crossingPoint]  = botSim.ultraScan() %perfoms simulated ultrasound scan
botSim.drawScanConfig(); %draws current scan configuration
botSim.drawBot(3);
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3); %draws crossingpoints

disp('As you can see, the measured distance straight down is 40cm, as you would expect.');
disp('The crossing point is marked as the blue O.');
input('Press enter to turn and take another scan');
disp('-------------------------------------------------------------------');
disp(' ');

botSim.turn(-pi/2)
[distances, crossingPoint]  = botSim.ultraScan() %performs a simulated scan

hold off; botSim.drawMap();  %resets drawing area
botSim.drawScanConfig();
botSim.drawBot(3);
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3);
% botSim.drawScanConfig();

disp('You can change the number of times the robot scans with the code:')
disp('botSim.setScanConfig(botSim.generateScanConfig(20)) sets the robot to');
input('scan 20 times all around the robot');
disp('-------------------------------------------------------------------');
disp(' ');

botSim.setScanConfig(botSim.generateScanConfig(20));  %sets the scan configuration on the
%botSim.  The other argument will be explained later.

clf; axis equal; hold on; botSim.drawMap();  %resets drawing area
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(3);

%You can now try scanning again with the new configuration
[distance crossingPoint]  = botSim.ultraScan()
%Now 20 distances and 20 crossing points are returned
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3);

disp('The distances to the wall are returned along with the intersection points');
disp('The scanLines will update automatically when you turn or move the robot');
input('Press enter to turn, move and scan again');
disp('-------------------------------------------------------------------');
disp(' ');

botSim.turn(-pi/4);
botSim.move(10);
clf; axis equal; hold on; botSim.drawMap();  %resets drawing area
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(3);
%You can now try scanning again with the new configuration
[distance crossingPoint]  = botSim.ultraScan()
%Now 20 distances and 20 crossing points are returned
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3);

disp('You can also set the sensor noise with the variable sensorNoise');
input('Press enter to repeat the scan with added sensor noise');
disp('-------------------------------------------------------------------');
disp(' ');

botSim.setSensorNoise(1);
clf; axis equal; hold on; botSim.drawMap();  %resets drawing area
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(3);
[distance crossingPoint]  = botSim.ultraScan()
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3);

disp(' ');
disp('Optional Extra');
disp('Advanced scanConfig setup (not required in most cases)');
disp('This will show you how to set up a scan that is not a full 360 degrees');
disp('or if the center of the scan is not the center of rotation for your bot');

disp(' ');
disp('For example, to scan pi/2 rad in 5 steps, construct a vector that goes');
disp('from -pi/4 to pi/4 (so that the scan is centered) with length 5')
input('press enter to generate angles');
disp('-------------------------------------------------------------------');
disp(' ');

botSim.setSensorNoise(0);
startAngle =-pi/4
endAngle = pi/4
samples = 5
angles= (startAngle:(endAngle - startAngle)/(samples-1):endAngle)
input('Then press enter to generate radial lines based on the generated angles');
disp('-------------------------------------------------------------------');
disp(' ');
scanLines =  [cos(angles); sin(angles)]'*100  %This produces n 2d vectors around the origin.
% Each vector is the direction of the scan.  The *100 factor on the end is not required
%but makes it easier to see when drawing.

disp('You can set the offset of the center of rotation of the scan as well');
scanOffSet = [4 0]  %This sets the center of the scan to [4 0] relative to
%the center of rotation of the robot

disp('Set a custom scan configuration with the function:');
disp('botSim.setScanConfig(scanLines,scanOffSet)');
botSim.setScanConfig(scanLines,scanOffSet);
botSim.turn(-pi/2);

input('Press enter to perform a custom scan');
disp('-------------------------------------------------------------------');
disp(' ');
hold off; botSim.drawMap();  %resets drawing area
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(3);
[distance crossingPoint]  = botSim.ultraScan()

%displays crossingPoints
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3);
disp('Example2 finished');