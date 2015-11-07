clc
clear
close all

notch = [0,0; 0,1; 0.5,0.4; 1,1; 1,0];
cshape = [0,0; 0,1; 0.5,1; 0.5,0.5; 1,0.5; 1,1; 1.5,1; 1.5,0];
map = Map(cshape, [0.25, 0.75], 0);
map.position = [1.25, 0.75];

figure();
map.plot();

while 1
    [bearing, distance] = map.findBearing();
    map.position = map.position + [cos(bearing), sin(bearing)] * 0.1;
    map.plot();
    pause();
end