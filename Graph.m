% Printing algorithm adapted from
% https://www.geeksforgeeks.org/find-paths-given-source-destination/
classdef Graph < handle 
    % 'handle' keyword ensures any member functions that act on an object
    % of type Graph pass the the object by reference. That way, the
    % specific object that calls the method will be changed, not a copy of
    % it
    properties
        m_vertices
        m_graph
        m_test
        m_vertexList = [];
        m_visitedVertices
        % Cell array to store all paths from vertex 1 to vertex 2:
        m_currentVertexPaths = {}; 
    end
   
    methods
        
        % Constructor
        function self = Graph(v)
            
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
            % Note that adjacency matrices must be square, so we can 
            % take just its number of columns to be its number of rows as
            % well:
            N = length(adjMatrix(:,1));
            for i = 1:N
                for j = 1:N
                    if (adjMatrix(i,j) == 1)
                        self.addEdge(i,j);
                    end
                end
            end
            
        end
        
        % Recursive helper function used to actually print and store paths
        function [self] = printAllPathsUtil(self, currentNode, ...
                                          goalNode, path)
                                      
            % Mark the current node as visited and store it in our path
            self.m_visitedVertices(currentNode) = true;
            path = [path currentNode];
            if currentNode == goalNode
                disp(path) % Comment this line to stop path printing
                % Append completed path to cell array of all paths:
                self.m_currentVertexPaths{end+1} = path;
            % If current vertex is not the destination, recur for all
            % vertices adjacent to this vertex
            else
                for i = 1:length(self.m_graph(currentNode))
                    % We want to iterate over the vertices connected
                    % to the current node. Since MATLAB doesn't have
                    % 'for i in thing' like Python, we have to use this
                    % trick to actually use the vertices connected to
                    % the current node:
                    temp = self.m_graph(currentNode);
                    val = temp(i);
                    if self.m_visitedVertices(val) == false
                        self.printAllPathsUtil(val, goalNode, path);
                    end
                end
            end
            % Remove current vertex from path and mark it as unvisited
            path = path(1:end-1);
            self.m_visitedVertices(currentNode) = false;
            
        end
        
        % Kicks off recursive helper function
        function [self] = printAllPaths(self, vertex1, vertex2)
            
            % Mark all vertices as not vistied:
            self.m_visitedVertices = false(1,self.m_vertices);
            % create an array for algorithm to fill up with vertices
            % to actually find paths (not store them like currentVertexPaths):
            path = [];
            % Call the recursive helper function to print all paths:
            self.printAllPathsUtil(vertex1, vertex2, path);
            
        end  
    end
end
