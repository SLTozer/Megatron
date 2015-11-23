function [moveCommands turnCommands] = pathPlan(currentPos,currentAng,target,map)
%Worst pathplanning function ever.  Assumes there are no obstacles and
%generates random movment instructions
numOfMoves = 5;
moveCommands = zeros(1,numOfMoves);
turnCommands = zeros(1,numOfMoves);
for i = 1:numOfMoves
   moveCommands(i) = rand(1)*10; 
   turnCommands(i) = rand(1)*pi/2; 
end
end