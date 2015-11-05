% This function calculates the shortest path from each given location in the
% adjacency matrix A to the target node, using Dijkstra's Algorithm for graph
% searching/traversal
% dist(i) is the distance from the ith node to the target
% next(i) is the next node in the shortest path from the ith node to the target
function [next,dist] = dijkstraChain(A,target)


for i = 1:length(A)
    dist(i) = inf(1,1);
    next(i) = -1;
end

dist(target) = 0;

for i = 1 :length(A)
    status(i) = 0;
end
checked = 0;

while (length(A) > checked)

    minElem = inf;
    for j = 1:length(A)
        if (status(j) ~= 1 && dist(j) < minElem)
            minElem = dist(j);
            mindex = j;
        end
    end

    for j = 1:length(A)
        if (A(mindex,j) > 0 && j ~= mindex)
            alt = dist(mindex) + A(mindex,j)

            if alt < dist(j)
                dist(j) = alt
                next(j) = mindex;
            end
        end
    end

    status(mindex) = 1;
    checked = checked + 1;

end

end