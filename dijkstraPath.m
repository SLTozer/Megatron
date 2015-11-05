% This function calculates the shortest path from the robot's perceived location
% to the target point, using Dijkstra's Algorithm for graph searching/traversal
function [path,dist] = dijkstraPath(A,source,target)


for i = 1:length(A)
    dist(i) = inf(1,1);
    prev(i) = -1;
end

dist(source) = 0;


for i = 1 :length(A)
    v(i) = i;
end

while (length(A) > 1)

    [minElem,mindex] = min(dist);
    vdex = v(mindex)

    for j = 1:length(A)
        jdex = v(j)
        if (A(mindex,j) > 0 && j ~= mindex)
            alt = dist(vdex) + A(mindex,j)

            if alt < dist(jdex)
                dist(jdex) = alt
                prev(jdex) = vdex;
            end
        end
    end

    A(mindex,:) = [];
    A(:,mindex) = [];
    v(mindex) = [];

end

j = target;
i = 1;

while(prev(j) ~= -1)
    path(i) = j;
    j = prev(j);
    i = i + 1;
end

path = fliplr(path);

end