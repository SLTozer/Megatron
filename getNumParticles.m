function [ num ] = getNumParticles( map, radius, numScans, prob )
% Returns the number of particles required to have at least 1 particle
% within 'radius' distance of the actual robot, with an orientation
% approximately equal to the robot's, with probability 'prob'

roboArea = pi * (radius^2);
mapArea = polyarea(map(:,1),map(:,2));

num = ceil((mapArea / roboArea) * numScans * (-log(1-prob)));
end

