clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

disp('You can use the pointInsideMap() function to sample the map vectors');
disp('to make a grid representation of the map for pathfinding');

hold on;
botSim.drawMap();

limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
res = 10; %sampling resouloution in cm
iterators = dims/res;
iterators = ceil(iterators)+[1 1]; %to counteract 1 based indexing
mapArray = zeros(iterators); %preallocate for speed
hold on
%loops through the grid indexes and tests if they are inside the map
for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing
        %notice, that i and j have also been swapped here so that the only
        %thing left to do is invert the y axis. 
        mapArray(i,j) = botSim.pointInsideMap(testPos);
        if mapArray(i,j)
            plot(testPos(1),testPos(2),'*');%inside map
        else
            plot(testPos(1),testPos(2),'o');%outside map
        end
    end
end

%prints array of samplePoints. 
disp('This naive implementation does not return the map in the same'); 
disp('orientation as plotted map.  Here the y axis is inverted')
mapArray

disp('You can also use the pointInsideMap() function to test if a vector of');
disp('points in inside a map. Each row of the n by 2 input represents a point.');
disp('The function will return 1 or 0 for each line to say if that point is inside the map');
%you can also use this in a vector form
testPoints = [0 0;80 40;20 20;-1 -1]
inside = botSim.pointInsideMap(testPoints)
disp('Example3 finished');