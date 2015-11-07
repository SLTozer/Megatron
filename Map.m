%{
    Map and target are given as co-ordinates [x, y].
    Position must be set but can be changed throughout.
    Calls to dijkstraRoute create chain of route to desired target.
%}
classdef Map < handle
    properties
        vertices
        polygonX
        polygonY
        walls
        dist
        position
    end
    methods
        function map = Map(map_in, target, robot_radius)
            % store map as list of vertices [x1, y1; x2, y2] and in format for "inpolygon"
            % call later [x1; x2] [y1; y2]
            % Polygons must have the first point as the last point to close the loop
            map.vertices = Map.shrinkMap(map_in, robot_radius);
            map.polygonX = cat(1,map.vertices(:,1), map.vertices(1,1));
            map.polygonY = cat(1,map.vertices(:,2), map.vertices(1,2));
            
            % create list of walls in format [x0, y0, x1, y2], one line per
            % row of array
            map.walls = Map.findWalls(map.vertices);
            
            % add target to vertices and solve for valid distances (including walls)
            map.vertices = [target; map.vertices];
            N = size(map.vertices,1);
            map.dist = zeros(N,N);
            for a = 1:N
                map.dist(a, a+1:N) = map.findValidDistances(map.vertices(a,:), map.vertices(a+1:N,:));
                map.dist(a+1:N, a) = map.dist(a, a+1:N);
            end
            
        end
        
        % Creates array of all valid distances between nodes
        % paths are valid if centre is within shape and it doesn't
        % intersect any walls.
        % Walls themselves are valid paths.
        function dist = findValidDistances(map, vert, vertices)
            N = size(vertices,1);
            dist = zeros(1,N);
            for a = 1:N
                p = [vert, vertices(a,:)];
                mid = (vert + vertices(a,:)) / 2;
                % Inpolygon supports concave polygons
                if inpolygon(mid(1),mid(2),map.polygonX,map.polygonY) && ~Map.intersects(map.walls, p) 
                    dist(a) = sqrt( (p(1) - p(3)) .^2 + (p(2) - p(4)) .^ 2 );
                end
            end
        end
        
        % Find bearing towards target (staying inside map). Empty array if
        % at target or no route found.
        function [bearing, distance] = findBearing( map )
            % where index [1:end-1] equate to vertices ([1] being the target)
            % index [end] equates to position
            pdist = map.findValidDistances(map.position, map.vertices);
            adj = [[map.dist; pdist(1:end)], [pdist, 0]'];
            [next, distances] = Map.dijkstraChain(adj, 1);
            if ~isequal(map.position, map.vertices(1,:))
                x = map.vertices(next(end),1) - map.position(1);
                y = map.vertices(next(end),2) - map.position(2);
                bearing = atan2( y, x);
            else
                bearing = [];
            end
            distance = distances(end);
        end
        
        function plot( map )
            clf
            hold on
            % plot planned route
            pdist = map.findValidDistances(map.position, map.vertices);
            adj = [[map.dist; pdist(1:end)], [pdist, 0]'];
            next = Map.dijkstraChain(adj, 1);
            last = map.position;
            i = next(end);
            while i ~= -1
                plot( [last(1), map.vertices(i,1)], [last(2), map.vertices(i,2)], 'r' );
                last = map.vertices(i,:);
                i = next(i);
            end
            % walls, target and position
            scatter(map.vertices(1,1), map.vertices(1,2), 'x')
            scatter(map.position(1), map.position(2), 'o', 'r')
            plot(map.polygonX, map.polygonY)
        end
    end
    methods(Static)        
        % returns true if segments B interesects and of A (EXCLUDING end points)
        % returns false if colinear
        % lines are defined by array [x0, y0, x1, y1]
        function bool = intersects(A, B)
            for i = 1: size(A,1)
                CAcrossCD = ((A(i,1)-B(1))*(B(4)-B(2))-(A(i,2)-B(2))*(B(3)-B(1)));
                CBcrossCD = ((A(i,3)-B(1))*(B(4)-B(2))-(A(i,4)-B(2))*(B(3)-B(1)));
                ACcrossAB = ((B(1)-A(i,1))*(A(i,4)-A(i,2))-(B(2)-A(i,2))*(A(i,3)-A(i,1)));
                ADcrossAB = ((B(3)-A(i,1))*(A(i,4)-A(i,2))-(B(4)-A(i,2))*(A(i,3)-A(i,1)));
                bool = ( sign(CAcrossCD * CBcrossCD) < 0 ) && ( sign(ACcrossAB * ADcrossAB) < 0);
                if bool
                    return
                end
            end
        end
        
        % return point of intersection (if any) of sqement P->P+r and Q->Q+s
        % NB if lines are colinear will not return a value!
        function point = intersection(P, r, Q, s)
            denom = Map.cross(r, s);
            enum = Map.cross(Q - P, r);
            if denom == 0 
                if enum == 0
                    % colinear
                    point = [];
                else
                    % parallel
                    point = [];
                end
            else
                t = Map.cross( Q - P, s ) / denom;
                u = enum / denom;
                if 0 <= t && t <= 1 && 0 <= u && u <= 1
                    point = P + t * r;
                else
                    % no intersection (but skew)
                    point = [];
                end
            end
        end
        
        % two dimensional cross product
        function prod = cross(A,B)
            prod = A(1) * B(2) - A(2) * B(1);
        end
        
        % create list of walls in format [x0, y0, x1, y2], one line per
        % row of array
        function walls = findWalls(vertices)
            walls = zeros(length(vertices),4);
            vertices(end+1,:) = vertices(1,:);
            for i =1:size(walls,1)
                walls(i,:) = [vertices(i,:) vertices(i+1,:)] ;
            end
        end
        
        % Returns vertices formed by shrinking existing shape by [quant].
        % The map is shrunk such that the new walls are [quant] inwards
        % from the old walls, in the direction normal to the old wall.
        function newvert = shrinkMap( vert, quant )
            newvert = zeros(size(vert));
            last_vec = vert(1,:) - vert(end,:);
            last_point = vert(end,:) + Map.normal(last_vec) * quant;
            vert(end+1,:) = vert(1,:); % add final value to end for fudged looping
            for v = 1:length(vert)-1
                vec = vert(v+1,:) - vert(v,:);
                point = vert(v,:) + Map.normal(vec) * quant;
                newvert(v,:) = Map.intersection(last_point, last_vec, point, vec);
                last_vec = vec;
                last_point = point;
            end
        end
        
        % produces the right hand unit normal to a line.
        function normvec = normal(A)
            normvec = (A * [0,-1;1,0]) / norm(A);
        end
        
        % This function calculates the shortest path from each given location in the
        % adjacency matrix A to the target node, using Dijkstra's Algorithm for graph
        % searching/traversal
        % dist(i) is the distance from the ith node to the target
        % next(i) is the next node in the shortest path from the ith node to the target
        function [next, dist] = dijkstraChain(A,target)
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
                        alt = dist(mindex) + A(mindex,j);

                        if alt < dist(j)
                            dist(j) = alt;
                            next(j) = mindex;
                        end
                    end
                end

                status(mindex) = 1;
                checked = checked + 1;
            end
        end
    end
end