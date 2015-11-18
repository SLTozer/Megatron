clc
clear
close all

Map.intersects([0,1,1,2], [0,0,1,1]) % false: parallel
Map.intersects([0,1,1,1.5], [0,0,1,1]) % false: skew but not intersecting
Map.intersects([-1,-1,0,0], [0,0,1,1]) % false: same point shared
Map.intersects([0,-1,0,1], [0,0,1,1]) % true: crosses start or end point
Map.intersects([1,0,0,1], [0,0,1,1]) % true: intersection
Map.intersects([-1,-1,0.5,0.5], [0,0,1,1]) % true: colinear

notch = [0,0; 0,1; 0.5,0.4; 1,1; 1,0];
cshape = [0,0; 0,1; 0.5,1; 0.5,0.5; 1,0.5; 1,1; 1.5,1; 1.5,0];
map = Map(cshape, [0.15, 0.75], 0.1);
position = [1.25, 0.75];

figure();
map.plot(position);

while 1
    pause();
    [bearing, distance] = map.findBearing(position);
    position = position + [cos(bearing), sin(bearing)] * 0.05;
    map.plot(position);
end