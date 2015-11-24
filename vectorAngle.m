function [ angle ] = vectorAngle( vec )
% Returns the angle of the given vector, where [1 0] = 0 and [0 1] = pi/2

    angle = atan(vec(2)/vec(1));
    if vec(1) < 0
        angle = angle + pi;
    end

end

