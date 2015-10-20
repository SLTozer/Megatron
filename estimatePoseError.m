function [prob] = estimatePoseError(oldPose,oxpos,oypos,newPose,xpos,ypos,timeTaken,omega,v)

    if (xpos == ypos && oxpos == oypos)
        mu = 0; 
        xstar = 0.5 * (xpos + oxpos) + mu * (oypos - ypos);
        ystar = 0.5 * (ypos + oypos) + mu * (oxpos - xpos);
        angleChange = newPose-oldPose;

    else
        mu =((oxpos - xpos)*cos(oldPose) + (oypos - ypos)*sin(oldPose));
        mu = mu / ((oypos - ypos)*cos(oldPose) + (oxpos - xpos)*sin(oldPose));
        mu = 0.5 * mu;

        xstar = 0.5 * (xpos + oxpos) + mu * (oypos - ypos);
        ystar = 0.5 * (ypos + oypos) + mu * (oxpos - xpos);
        angleChange = atan2(ypos - ystar, xpos - xstar) - atan2(oypos - ystar, oxpos - xstar);
    end


    rstar = sqrt(power((oxpos - xstar),2) + power((oypos - ystar),2));

    vhat = (angleChange/timeTaken) * rstar;
    omegahat = (angleChange/timeTaken);
    gammahat = ((newPose-oldPose)/timeTaken) - omegahat;

    prob = normpdf(v - vhat, 0, 0.15*abs(v) + 0.15*abs(omega)); 
    prob = prob * normpdf(omega-omegahat,0, 0.15*abs(v) + 0.15*abs(omega)); 
    prob = prob * normpdf(gammahat,0, 0.15*abs(v) + 0.15*abs(omega));

end