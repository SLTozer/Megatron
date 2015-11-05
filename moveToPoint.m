function moveToPoint(currAngle,pos,tpos,robot)

xdist = tpos(1) - pos(1);
ydist = tpos(2) - pos(2);

distance = sqrt(xdist^2 + ydist^2);

newAngle = atan2(xdist,ydist);

turnAmount = currAngle - newAngle;

robot.turn(turnAmount);
robot.move(distance);
end