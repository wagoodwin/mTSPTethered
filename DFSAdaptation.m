
%% DFS Path Print Demo

clc;
clearvars;


% % METHOD 1
% 
% % Initialize graph object
% graph = Graph(4);
% 
% % Load in graph with standard method (.addEdge):
% 
% % note: with this notation, graph is actually passed as an argument 
% % ALONG with 1 and 2. So it's really like addEdge(graph,1,2).
% graph.addEdge(1, 2);
% graph.addEdge(1, 3);
% graph.addEdge(1, 4);
% graph.addEdge(3, 1);
% graph.addEdge(3, 2);
% graph.addEdge(2, 4);
% 
% % Print all paths between vertices:
% graph.printAllPaths(3,4);
% allPaths = graph.m_currentVertexPaths



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 
% METHOD 2

% bonusGraph = Graph(4);
% 
% % Load in graph with adjacency matrix:
% adjMatrix = [0 1 1 1; 0 0 0 1; 1 1 0 0; 0 0 0 0];
% bonusGraph.loadAdjacencyMatrix(adjMatrix);
% 
% bonusGraph.printAllPaths(3,4);
% allPathsBonus = bonusGraph.m_currentVertexPaths


%% Edge-DFS

graph2 = Graph2(4);

adjMatrix = [0 1 2 1; 0 0 0 1; 2 1 0 0; 0 0 0 0];

graph2.loadAdjacencyMatrix(adjMatrix);

paths = graph2.printAllPaths(3,4);


% If you want to access each path:
allPaths = graph2.m_visitedRoutes;

% The algorithm works with self-loop too. Can show 


% Make sure the loop can start and end at the same point MUST FIX!!!

%% Notes

% NOTE 1
% Current problem: every time we add an edge (do fcn .addEdge()),
% our member variable self.m_vertexList resets. That's why we can't 
% ever get keys with multiple values

% Specifically, graph.m_vertexList never seems to get filled up. WTF
% Solution: you have to specify if a class will pass stuff by value or by
% reference. It's pass by value by default. By adding '< handle', we were
% able to make the class pass by reference, fixing the problem.

% NOTE 2
% Fun announcement: 0-nodes will not be allowed. Because of MATLAB's
% 1-indexing, allowing a 0th node screws stuff up and would require some
% annoying changes to make things work. Also, it means that node n
% according to MATLB is actually node n-1, which is also a pain. For
% simplicity, 0-nodes are out. 

% NOTE 3
% Didn't implement super ideal OOP standards here. Really, the Graph
% object's members should be private, and we implement getter and setter
% methods to grab/modify those objects. Happy to implement that stuff if
% necessary.



