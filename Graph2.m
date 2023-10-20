
% Initial printing algorithm adapted from
% https://www.geeksforgeeks.org/find-paths-given-source-destination/
classdef Graph2 < handle 
    % 'handle' keyword ensures any member functions that act on an object
    % of type Graph pass the the object by reference. That way, the
    % specific object that calls the method will be changed, not a copy of
    % it
    properties
        m_vertices;
        m_graph;
        m_vertexList = [];
        m_adjMatrix = [];
        m_visitedRoutes = {};
        m_routeNumber = 1; % (number of total routes we find):
        m_currentNearestVertex;
    end
   
    methods
        
        % Constructor
        function self = Graph2(v)
            
            self.m_vertices = v;
            % initialize map data structure to hold graph. Key-value pairs.
            self.m_graph = containers.Map('KeyType','double','ValueType','any');
            
        end
        
        % Add an edge to a graph
        function self = addEdge(self, vertex1, vertex2)
            
            % If vertex 1 is part of the vertex list, its key already
            % exists, so we can add the value vertex2 to that key.
            if (ismember(vertex1, self.m_vertexList) == 1)
                % MATLAB-style way of appending multiple values to one key:
                self.m_graph(vertex1) = [self.m_graph(vertex1) vertex2];
            % Otherwise, make a new key-value pair:
            else
                self.m_graph(vertex1) = [vertex2];
                self.m_vertexList = [self.m_vertexList vertex1];
            end
            
        end
        
        % Initialize the object's graph with an adjacency matrix. Different
        % method as opposed to adding edges.
        function self = loadAdjacencyMatrix(self, adjMatrix)
            % First assign the adjMatrix to a member variable for later
            % use:
            self.m_adjMatrix = adjMatrix;
            % Now load in the graph:
            % Note that adjacency matrices must be square, so we can 
            % take just its number of columns to be its number of rows as
            % well:
            N = length(adjMatrix(:,1));
            for i = 1:N
                for j = 1:N
                    if (adjMatrix(i,j) >= 1)
                        self.addEdge(i,j);
                    else
                        % Otherwise, add an empty vector. This detail
                        % ensures the path printing algorithm can still
                        % iterate over a vertex that's not connected 
                        % to other vertices. That is, with the [], when
                        % the loop hits that node with no connections,
                        % it'll just skip over that node instead of 
                        % throwing an error. 
                        self.addEdge(i,[]);
                    end
                end
            end
            
        end
        
        % Clears all visited routes. Somewhat hacky fix to prevent tours
        % getting mixed together somehow when iterating over ordered pairs
        % (i,j).
        % LONG-TERM FIX: have the code reset this stuff after every run by
        % itself
        function [self] = resetObject(self)
            self.m_visitedRoutes = {};
            self.m_routeNumber = 1;
            self.m_currentNearestVertex = [];
        end
        
        % Recursive helper function used to actually print and store paths
        function [self] = printAllPathsUtil(self, goalNode, path)
                                      
            % Store the current node in the path:
            path = [path self.m_currentNearestVertex];

            % "For all nodes adjacent to the end of the path"
            for i = 1:length(self.m_graph(path(end)))
                
                % Note that before these lines execute, the current nearest
                % vertex and end of the path are the same. After these 
                % lines execute, they won't necessarily be the same. 
                temp = self.m_graph(path(end));
                self.m_currentNearestVertex = temp(i);
                
                routeBool = isCurrentRouteInVisitedRoutes(self.m_visitedRoutes, path);
                
                % If we're at the goal node AND we haven't stored this route
                % yet, DO THIS:
                if (path(end) == goalNode && routeBool == false)

%                     disp(path) % Comment this line to stop path printing
                    % Increment the total number of routes we've found:
                    self.m_routeNumber = self.m_routeNumber + 1;
                    % Add an extra empty slot in the set-of-sets of visited
                    % routes. Since we just found a route that works, that 
                    % route will take up the first set in this set-of-sets.
                    % Hence, add a space for the next route:
                    self.m_visitedRoutes{self.m_routeNumber} = ...
                        self.m_visitedRoutes{self.m_routeNumber-1};
                    % The new route starts where the last one left off
                % If current vertex is not the destination, recur for all
                % vertices adjacent to the current vertex:                 
                end

                % Now, first check to make sure we can go from our
                % current vertex to the next closest vertex: If that edge on 
                % the adj matrix is 0, that means all the edges
                % connecting the two vertices have been used in 
                % previous routes, so skip over it to the next vertex:
                if (self.m_adjMatrix(path(end), ...
                        self.m_currentNearestVertex) <= 0)
                    %{
                    % The technique of removing edges from the graph
                    % prevents us from getting an infinite loop.
                    % Without removing edges from the graph, the algo
                    % has no reason to ever stop, as it can always find
                    % routes that work. Removing edges for each layer
                    % deeper into the recursion calls now gives the robot
                    % fewer places to go, eventually stopping the madness.
                    %}
                        
                        
%                     assert(self.m_adjMatrix(path(end-1), path(end)) >= -0.1, ...
%                     "Negative value in Adjacency Matrix!")
%                     % Put -0.1 instead of 0 to enforce positive values
%                     % because -0.0000 will sometimes be returned for values
%                     % in the adjacency matrix, making the algo think a
%                     % negative value appeared. However, since all these
%                     % values are integers, -0.1 will never happen, so we'll
%                     % get real errors if an actual negative integer shows
%                     % up

                    % Move on to the next edge, and act like it 
                    % doesn't exist (go to next iteration in for
                    % loop)
                    continue;

                % Otherwise, if the adj matrix looks good (still have an
                % edge we can use), subtract 1 at the corresponding
                % edge on the adj matrix,
                else
                    self.m_adjMatrix(path(end), self.m_currentNearestVertex) = ...
                        self.m_adjMatrix(path(end), ...
                        self.m_currentNearestVertex) - 1;

                    % add that route to array of visited routes. Technically,
                    % we haven't visited this route yet. We're putting it in
                    % preemptively. 
                    self.m_visitedRoutes{self.m_routeNumber}{end+1} = ...
                        self.m_currentNearestVertex;

                    % And finally, jump into the next recursion:
                    self.printAllPathsUtil(goalNode, path);
                end
                
            end
            % Quick check: if the path has only one value, and that value
            % is the home node, it means we've exhausted all possible
            % paths. Hence, the algorithm is done, so we should return.
            if (numel(path) == 1)
                return 
            end

            % If we're at the goal node AND we haven't stored this route
            % yet, DO THIS:
            routeBool = isCurrentRouteInVisitedRoutes(self.m_visitedRoutes, path);
            if (path(end) == goalNode && routeBool == false)

%                 disp(path) % Comment this line to stop path printing
                % Increment the total number of routes we've found:
                self.m_routeNumber = self.m_routeNumber + 1;
                % Add an extra empty slot in the set-of-sets of visited
                % routes. Since we just found a route that works, that 
                % route will take up the first set in this set-of-sets.
                % Hence, add a space for the next route:
                self.m_visitedRoutes{self.m_routeNumber} = ...
                    self.m_visitedRoutes{self.m_routeNumber-1};
                % The new route starts where the last one left off
            % If current vertex is not the destination, recur for all
            % vertices adjacent to the current vertex:
            %{
                % We want to iterate over the vertices connected
                % to the current node. Since MATLAB doesn't have
                % 'for i in thing' like Python, we have to use this
                % trick to actually use the vertices connected to
                % the current node:
    %                     temp = self.m_graph(currentNode);
    %                     val = temp(i);


    %}                    
            end

            % Add route back to adjacency matrix,
            self.m_adjMatrix(path(end-1),path(end)) = ...
                self.m_adjMatrix(path(end-1), path(end)) + 1;
            % Remove current vertex from path. Why? Since we just got out
            % of a recursion here, we need to back up one vertex.
            path = path(1:end-1);
            % Remove from visited routes:
            self.m_visitedRoutes{self.m_routeNumber}(end) = [];

        end
        
        % Kicks off recursive helper function
        function [self] = printAllPaths(self, vertex1, vertex2)
            
            self.m_currentNearestVertex = vertex1;
            % Initialize visitedRoutes set-of-sets with first vertex:
            self.m_visitedRoutes{1}{1} = vertex1;
            % create an array for algorithm to fill up with vertices
            % to actually find paths (not store them like currentVertexPaths):
            path = [];
            % Call the recursive helper function to print all paths:
            self.printAllPathsUtil(vertex2, path);
            
        end
        
    end
    
end

% UTILITY FUNCTIONS

% Converts a cell array of normal cell arrays to a cell array of type 
% double cell arrays
function [output] = convertToCellOfDoubles(cellArray)

    output = cell(length(cellArray),1);
    for i = 1:length(cellArray)
        output(i) = {cell2mat(cellArray{i})};
    end

end

%
function [bool] = isCurrentRouteInVisitedRoutes(visitedRoutesArray, path)

    % The 1:end-1 is a technical detail. Since we add the next vertex on
    % to the list of visited routes in the algo technically BEFORE we've
    % officially visited it, it shouldn't be included in our check. 
    visitedRoutesTypeDbl = ...
        convertToCellOfDoubles(visitedRoutesArray(1:end-1))';
    % Get status on if current route is in the set of 
    % visited routes:
    boolVec =   cellfun(@isequal, visitedRoutesTypeDbl(:), repmat({path}, ...
             length(visitedRoutesTypeDbl), 1));
         
    bool = sum(boolVec);

end


        
% WORK OUT PROBLEM BY HAND AND STEP THROUGH WITH DEBUGGER AT THE SAME
% TIME. SHOULD GET SAME THING.

% When we build the map out of the graph, every vertex must be a key
% in the map. Here, even though vertex 4 isn't connected to any other 
% vertices, it still must have a key-value pair that looks like (4,[]). 
% Otherwise, when the algorithm gets to vertex 4 and uses it to index
% the graph for the loop (at start of else block), it won't have a value
% to index (not even 0), so you'll get an error. If you do (4,[]),
% then you get length(self.m_graph(4)) = 0, and with MATLAB, the loop
% will just automatically exit for a counter like i = 1:0. 



