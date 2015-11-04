clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map

disp('You can set up a new BotSim object with the code:');
disp('botSim = BotSim(map)');
disp(' ');
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

disp('As debug mode is on by default, you can set the angle and position of the robot');
disp('with setBotPos() and setBotAng(). During marking you will not be able');
input('to access these values.  Press enter to initialise the robot:');

botSim.setBotPos([20 40])
botSim.setBotAng(pi)
disp('-------------------------------------------------------------------');
disp(' ');
disp('The current pose of the robot relative to the map is currently displayed');
disp('in figure 1.');
disp('In debug mode you can get the current bot position with getBotPos()');
botPosition = botSim.getBotPos()
disp('You can get the current bot angle in radians with getBotAng()');
botAngle = botSim.getBotAng()
botSim.drawMap();
botSim.drawBot(3);

disp('You can use the function turn() to turn the robot');
input('Press enter to make the robot turn pi/4 rad (45 deg)');
disp('-------------------------------------------------------------------');
disp(' ');
botSim.turn(pi/4);
hold off;
botSim.drawMap();
botSim.drawBot(3);

disp('you can use the function move(), to move the robot');
disp('forwards and backwards');
disp(' ');
input('Press enter to make the robot move forward 10cm');
disp('-------------------------------------------------------------------');
disp(' ');
botSim.move(10);
botSim.drawBot(3);

disp('You can combine turning and moving by calling both functions')
disp(' ');
input('Press enter to make the robot turn -pi/4 rad and move forward 30cm');
disp('-------------------------------------------------------------------');
disp(' ');
botSim.turn(-pi/4);
botSim.move(30);
botSim.drawBot(3);
disp('Note how the robot is able to leave the arena, no collision checking')
disp('is performed. You can call botSim.insideMap() to check if it is inside the map:')
insideMap = botSim.insideMap()

input('Press enter to reverse by 20cm')
disp('-------------------------------------------------------------------');
disp(' ');
botSim.move(-20);
botSim.drawBot(3);

disp('The insideMap function now returns 1')
insideMap = botSim.insideMap()

disp('You can set the motion noise level with the function setMotionNoise()');
input('Press enter to show the robot move from one location 50 times');
disp('-------------------------------------------------------------------');
disp(' ');
botSim.setBotPos([25 10]);
botSim.setBotAng(pi/2);
hold off
botSim.drawMap();
botSim.drawBot(3);

botSim.setMotionNoise(0.1);

for i = 1:50
    botSim.setBotPos([25 10]);
    botSim.move(60);
    botSim.drawBot(3);
end

disp('You can also set the angular noise with the function setTurningNoise()');
input('Press enter to show the robot move from one location 50 times');
disp('-------------------------------------------------------------------');
disp(' ');

botSim.setBotPos([25 10]);
botSim.setBotAng(pi/2);

hold off
botSim.drawMap();
botSim.drawBot(3);


botSim.setTurningNoise(0.05);
for i = 1:50
    botSim.setBotPos([25 10]);
    botSim.setBotAng(pi/2);
    botSim.move(60);
    botSim.drawBot(3);
end
disp('Example1 finished');