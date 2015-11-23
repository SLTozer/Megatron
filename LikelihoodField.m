%{
    Map is given as co-ordinates [x, y].
    All likelihood values are assigned on construction and do not change.
    Grid size and width are static.
    Likelihood values can be looked up from raw 2d coordinates.
%}
classdef LikelihoodField < handle
    properties
        field
        fieldSize
        minPoint
        maxPoint
        cellWidth
        minLikelihood
    end
    methods

        function lf = LikelihoodField(map_in, width, falloff, minLikelihood)
            % Store cell likelihood values as 2d array 'field' of size 'fieldSize'
            % 'minPoint' is the most negative corner of the map's axis-aligned bounding box
            % field(x,y) is the likelihood value at minPoint + ([x-1,y-1]*cellWidth)
            % 'falloff' is the rate of decay in likelihood of distance to an obstacle (must be <1)
            %
            % Calculates the minimum distance (in cells) between a cell with 0 likelihood and a wall
            % Likelihood lower than 'minLikelihood' is ignored
            lf.minLikelihood = minLikelihood;
            falloffDist = log(minLikelihood)/log(falloff);
            lf.cellWidth = width;
            % Calculates the axis-aligned bounding box of the map
            lf.minPoint = [Inf,Inf];
            lf.maxPoint = [-Inf,-Inf];
            N = size(map_in,1);
            for a = 1:N
                for d = 1:2
                    if map_in(a,d) < lf.minPoint(d)
                        lf.minPoint(d) = map_in(a,d);
                    elseif map_in(a,d) > lf.maxPoint(d)
                        lf.maxPoint(d) = map_in(a,d);
                    end
                end
            end
            lf.minPoint = lf.minPoint - falloffDist;
            lf.maxPoint = lf.maxPoint + falloffDist;
            % Calculates the dimensions of the field
            lf.fieldSize = ceil((lf.maxPoint - lf.minPoint) / width) + 1;
            lf.field = ones(lf.fieldSize) * minLikelihood;
            % Calculates likelihood values for the field
            for a = 1:N
                % Get wall bounding box
                if a == N
                    b = 1;
                else
                    b = a + 1;
                end
                wall = [map_in(a,:);map_in(b,:)];
                wallMinCoord = wall(1,:);
                wallMaxCoord = wall(1,:);
                for d = 1:2
                    if wall(2,d) < wallMinCoord(d)
                        wallMinCoord(d) = wall(2,d);
                    elseif wall(2,d) > wallMaxCoord(d)
                        wallMaxCoord(d) = wall(2,d);
                    end
                end
                wallMinCoord = wallMinCoord - falloffDist;
                wallMaxCoord = wallMaxCoord + falloffDist;
                wallMinPoint = lf.findNearestPoint(wallMinCoord);
                wallMaxPoint = lf.findNearestPoint(wallMaxCoord);
                % Assign likelihood values
                for x = wallMinPoint(1):wallMaxPoint(1)
                    for y = wallMinPoint(2):wallMaxPoint(2)
                        dist = LikelihoodField.distPointToSegment(lf.getPointCoords([x,y]), wall(1,:), wall(2,:));
                        pointLikelihood = falloff^dist;
                        if lf.field(x,y) < pointLikelihood
                            lf.field(x,y) = pointLikelihood;
                        end
                    end
                end
            end
        end
        
        % Returns the [x y] of the nearest field cell to the given coords
        function point = findNearestPoint(lf, coords)
            clampedCoords = coords;
            for d = 1:2
                if clampedCoords(d) < lf.minPoint(d)
                    clampedCoords(d) = lf.minPoint(d);
                elseif clampedCoords(d) > lf.maxPoint(d)
                    clampedCoords(d) = lf.maxPoint(d);
                end
            end
            relativeCoords = clampedCoords - lf.minPoint;
            point = round(relativeCoords/lf.cellWidth) + 1;
        end

        % Returns the 2d coordinates corresponding to field(p)
        function point = getPointCoords(lf, p)
            point = lf.minPoint + ((p-1)*lf.cellWidth);
        end

        % Returns the likelihood for the given 2d coordinates
        function prob = getPointLikelihood(lf, coords)
            point = lf.findNearestPoint(coords);
            prob = lf.field(point(1),point(2));
        end

    end
    methods(Static)

        % Returns the shortest distance from the point p to the line segment between points a & b
        function dist = distPointToSegment(p, a, b)
            if dot(b-a, p-a) * dot(a-b, p-b) >= 0
                dist = abs(det([a,1; b,1; p,1]))/norm(b-a);
            else
                dist = min(norm(p-a), norm(p-b));
            end
        end

    end
end
