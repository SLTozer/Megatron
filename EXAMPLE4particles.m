clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

disp('This example shows a simple procedure for updating your particles');
disp('First create a vector with N botSim object, one for each particle');

num =200;

botSim.drawMap();
particles(num,1) = BotSim; %how to set up a vector of objects
for i =1:num
    particles(i) = BotSim(map);
    particles(i).randomPose(10); 
    particles(i).drawBot(3);
end

input('Press enter to send the same movement commands to each particle');
hold off;
botSim.drawMap();
disp('-------------------------------------------------------------------');
disp(' ');
for i =1:num
    particles(i).turn(0.1);
    particles(i).move(20);
    particles(i).drawBot(3);
end

disp('Remove bots outside the map and generate a new random particle.');
input('This is where you would write your resampling function');
disp('-------------------------------------------------------------------');
disp(' ');


hold off;
botSim.drawMap();
for i =1:num    
    if particles(i).insideMap() == 0
        particles(i).randomPose(0); %at least 5cm from the wall
    end
    particles(i).drawBot(3);
end

disp('');
disp('Example4 finished');
