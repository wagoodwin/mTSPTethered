
% Printing algorithm adapted from
% https://www.geeksforgeeks.org/find-paths-given-source-destination/
classdef Graph2 < handle 
    % 'handle' keyword ensures any member functions that act on an object
    % of type Graph pass the the object by reference. That way, the
    % specific object that calls the method will be changed, not a copy of
    % it
    properties
        m_vertices;
        m_graph;
        m_test;
        m_vertexList = [];
        % Cell array to store all paths from vertex 1 to vertex 2:
        m_currentVertexPaths = {};
        m_adjMatrix = [];
        m_visitedRoutes = {};
        m_routeNumber = 1; % (number of total routes we find):
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
                        self.addEdge(i,[]);
                    end
                end
            end
            
        end
        
        % Recursive helper function used to actually print and store paths
        function [self] = printAllPathsUtil(self, currentRoute, ...
                                          goalNode, path)
                                      
            % Store the current node in the path (don't mark it as visited):
            path = [path currentRoute(end)];

            for i = 1:length(self.m_graph(path(end)))
                temp = self.m_graph(path(end));
                currentNearestVertex = temp(i);
                
                swag = convertToCellOfDoubles(self.m_visitedRoutes(1:end-1))';
                % Get status on if current route is in the set of 
                % visited routes:
                isCurrentRouteInVisitedRoutes = ...
                         cellfun(@isequal, swag(:), repmat({path}, length(swag), 1));
                    
                isCurrentRouteInVisitedRoutes = sum(isCurrentRouteInVisitedRoutes);
%                                         cellfun(@isequal, swag(:), repmat({path}, ...
%                     length(self.m_visitedRoutes), 1));
                % IDEA: WHY NOT JUST LOOK AT ALL ROUTES BUT THE MOST
                % CURRENT ONE FOR SELF.M_VISITEDROUTES?


            
                % If the list of visited routes is empty (like at
                % initialization), just set the status equal to false:
                if( isempty(self.m_visitedRoutes(1:end-1)) == true)
                    isCurrentRouteInVisitedRoutes = false;
                end
                
                % If we're at the goal node AND we haven't stored this route
                % yet, DO THIS:
                if (path(end) == goalNode && isCurrentRouteInVisitedRoutes == false)

                    disp(path) % Comment this line to stop path printing
                    % Append completed path to cell array of all paths:
                    self.m_currentVertexPaths{end+1} = path;
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


                    % NOW NEED TO GET RELATIONSHIP BETWEEN ADJ MATRIX 
                    % AND GRAPH. THAT IS, WHEN THE ADJ MATRIX REACHES A 
                    % A CERTAIN POINT, THE GRAPH NEEDS TO CHANGE IN A
                    % CERTAIN WAY. THAT IS, NEED TO REMOVE AN EDGE FROM
                    % THE GRAPH WHEN THAT EDGE IN THE ADJ MATRIX BECOMES 0
                    % NOTE QUITE SURE YET HOW TO DO THAT


                    % NEW: IF THE NODE HAS BEEN VISITED BY A CERTAIN
                    % PREVIOUS TOUR, THEN RUN FUNCTION AGAIN. THAT IS, WE
                    % COME UP TO THIS NODE. THEN, WE COMPARE OUR CURRENT
                    % TOUR THE TOURS THAT HAVE GONE TO THIS NODE. IF OUR
                    % CURRENT TOUR IS NOT ONE OF THOSE TOURS THAT HAVE GONE
                    % TO IT, WE'RE GOOD. IF NOT, NO RECURSION
                    %
    %}                    
                end

                % Now, first check to make sure we can go from our
                % current vertex to the next closest vertex: If that edge on 
                % the adj matrix is 0, that means all the edges
                % connecting the two vertices have been used in 
                % previous routes, so skip over it to the next vertex:
                if (self.m_adjMatrix(path(end), ...
                        currentNearestVertex) <= 0)
                    %{
                    % The technique of removing edges from the graph
                    % prevents us from getting an infinite loop.
                    % Without removing edges from the graph, the algo
                    % has no reason to ever stop, as it can always find
                    % routes that work. Removing edges for each layer
                    % deeper into the recursion calls now gives the robot
                    % fewer places to go, eventually stopping the madness.
                    %}
                    assert(self.m_adjMatrix(path(end-1), path(end)) >= 0, ...
                    "Negative value in Adjacency Matrix!")

                    % Move on to the next edge, and act like it 
                    % doesn't exist (go to next iteration in for
                    % loop)
                    continue;
                end
                % Note that we only enter the loop if currentRoute
                % has more than one value. If it only has one
                % value (like at initialization), it hasn't taken
                % an edge yet, so the adjacency matrix doesn't need
                % to be decremented.

                % If the adj matrix is looking good (still have an
                % edge we can use), subtract 1 at the corresponding
                % edge on the adj matrix,

                self.m_adjMatrix(path(end), currentNearestVertex) = ...
                    self.m_adjMatrix(path(end), ...
                    currentNearestVertex) - 1;

                %{
                % If that entry on the adj matrix is 0, remove the 
                % corresponding edge from the graph:
                % SEE IF YOU CAN GET AWAY WITH NOT DELETING AN
                % EDGE. INSTEAD, TRY JUST DOING CONTINUE IN THE IF
                % STATEMENT. THEN, WE'LL JUST MOVE TO THE NEXT
                % NODE, CONCEPTUALLY.


%                         if (self.m_adjMatrix(currentRoute(end-1,end)) <= 0)
%                             %{
%                             % The technique of removing edges from the graph
%                             % prevents us from getting an infinite loop.
%                             % Without removing edges from the graph, the algo
%                             % has no reason to ever stop, as it can always find
%                             % routes that work. Removing edges for each layer
%                             % deeper into the recursion calls now gives the robot
%                             % fewer places to go, eventually stopping the madness.
%                             %}
%                             assert(self.m_adjMatrix(currentRoute(end-1,end)) < 0, ...
%                             "Negative value in Adjacency Matrix!")
%                         
%                             temp = self.m_graph(currentRoute(end));
%                             temp = temp(temp ~= currentRoute(end)); % rm vertex here
%                             self.m_graph(currentRoute(end)) = temp; % reassign to graph
%                         end
                %}
                % add that route to array of visited routes. Technically,
                % we haven't visited this route yet. We're putting it in
                % preemptively. 
                self.m_visitedRoutes{self.m_routeNumber}{end+1} = currentNearestVertex;

                % Add vertex on to current route:
                currentRoute = [currentRoute, currentNearestVertex];

                % And finally, jump into the next recursion:
                self.printAllPathsUtil(currentRoute, goalNode, path);
            end
            % Quick check: if the path has only one value, and that value
            % is the home node, it means we've exhausted all possible
            % paths. Hence, the algorithm is done, so we should return.
            if (numel(path) == 1) % INVESTIGATE THIS PART. MAYBE PUT CONTINUE HERE
                return % MIGHT BE ABLE TO DELETE THIS WHOLE CHUNK
            end
            
            
                            
                        swag = convertToCellOfDoubles(self.m_visitedRoutes(1:end-1))';
                        % Get status on if current route is in the set of 
                        % visited routes:
                        isCurrentRouteInVisitedRoutes = ...
                                 cellfun(@isequal, swag(:), repmat({path}, length(swag), 1));

                        isCurrentRouteInVisitedRoutes = sum(isCurrentRouteInVisitedRoutes);
        %                                         cellfun(@isequal, swag(:), repmat({path}, ...
        %                     length(self.m_visitedRoutes), 1));
                        % IDEA: WHY NOT JUST LOOK AT ALL ROUTES BUT THE MOST
                        % CURRENT ONE FOR SELF.M_VISITEDROUTES?



                        % If the list of visited routes is empty (like at
                        % initialization), just set the status equal to false:
                        if( isempty(self.m_visitedRoutes(1:end-1)) == true)
                            isCurrentRouteInVisitedRoutes = false;
                        end

                        % If we're at the goal node AND we haven't stored this route
                        % yet, DO THIS:
                        if (path(end) == goalNode && isCurrentRouteInVisitedRoutes == false)

                            disp(path) % Comment this line to stop path printing
                            % Append completed path to cell array of all paths:
                            self.m_currentVertexPaths{end+1} = path;
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


                            % NOW NEED TO GET RELATIONSHIP BETWEEN ADJ MATRIX 
                            % AND GRAPH. THAT IS, WHEN THE ADJ MATRIX REACHES A 
                            % A CERTAIN POINT, THE GRAPH NEEDS TO CHANGE IN A
                            % CERTAIN WAY. THAT IS, NEED TO REMOVE AN EDGE FROM
                            % THE GRAPH WHEN THAT EDGE IN THE ADJ MATRIX BECOMES 0
                            % NOTE QUITE SURE YET HOW TO DO THAT


                            % NEW: IF THE NODE HAS BEEN VISITED BY A CERTAIN
                            % PREVIOUS TOUR, THEN RUN FUNCTION AGAIN. THAT IS, WE
                            % COME UP TO THIS NODE. THEN, WE COMPARE OUR CURRENT
                            % TOUR THE TOURS THAT HAVE GONE TO THIS NODE. IF OUR
                            % CURRENT TOUR IS NOT ONE OF THOSE TOURS THAT HAVE GONE
                            % TO IT, WE'RE GOOD. IF NOT, NO RECURSION
                            %
            %}                    
                        end

            % Add route back to adjacency matrix,
            self.m_adjMatrix(path(end-1),path(end)) = ...
                self.m_adjMatrix(path(end-1), path(end)) + 1;
            % Remove current vertex from path
            path = path(1:end-1);
%             % Remove from visited routes:
%             self.m_visitedRoutes{self.m_routeNumber} = ...
%                 self.m_visitedRoutes{self.m_routeNumber}{1:end-1};
            self.m_visitedRoutes{self.m_routeNumber}(end) = [];
            % REMOVE ADDING EDGES BACK TO THE GRAPH 
            % And add edge back to graph:
%             self.m_graph(currentRoute(end-1)) = [self.m_graph(end-1) self.m_graph(end)]; 
            
        end
        
        % Kicks off recursive helper function
        function [self] = printAllPaths(self, vertex1, vertex2)
            
            currentRoute = [vertex1];
            % Initialize visitedRoutes set-of-sets with first vertex:
            self.m_visitedRoutes{1}{1} = vertex1;
            % create an array for algorithm to fill up with vertices
            % to actually find paths (not store them like currentVertexPaths):
            path = [];
            % Call the recursive helper function to print all paths:
            self.printAllPathsUtil(currentRoute, vertex2, path);
            
        end
    end
end


function [output] = convertToCellOfDoubles(cellArray)

    output = cell(length(cellArray),1);
    for i = 1:length(cellArray)
        output(i) = {cell2mat(cellArray{i})};
    end

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



