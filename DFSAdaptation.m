
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

% graph2 = Graph2(4);
% 
% adjMatrix = [0 1 2 1; 0 0 0 1; 2 1 1 0; 0 0 0 0];
% 
% graph2.loadAdjacencyMatrix(adjMatrix);
% 
% paths = graph2.printAllPaths(3,4);
% 
% 
% % To access each path:
% allPathsFromHome = graph2.m_visitedRoutes;




% Here's an example of the algorithm working with a real adjacency matrix.

matrix =   [0     1     0     0     0     0     0     0     0     0   0;
            1     0     0     0     0     0     0     1     0     0   0;
            0     0     0     1     0     0     0     1     0     0   0;
            0     0     1     0     1     0     0     0     0     0   0;
            0     0     0     1     0     0     0     0     0     0   0;
            0     0     0     0     0     0     0     0     0     0   0;
            0     0     0     0     0     0     0     0     0     0   0;
            0     1     1     0     0     0     0     0     0     0   0;
            0     0     0     0     0     0     0     0     0     0   0;
            0     0     0     0     0     0     0     0     0     0   0;
            0     0     0     0     0     0     0     0     0     0   0];

numVertices = numel(matrix(:,1));
graph2Mtsp = Graph2(numVertices);
graph2Mtsp.loadAdjacencyMatrix(matrix);

paths2 = graph2Mtsp.printAllPaths(1,1);
allPaths = graph2Mtsp.m_visitedRoutes;

% This adjacency matrix doesn't have any repeat paths, but as an easy
% demo, put a 2 at the (1,1) spot of the matrix and run the sim again. 

%% test

% USE DIFFERENT LANGUAGE than citiesVisited and citiesUnvisited. They're
% all visited; it's just that some are parts of subtours and some aren't.


numCities = 6;
clc

% yili = [1 0 0 0; 0 0 0 0; 0 1 0 1; 0 1 0 0];
yili = [1 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 1 0 0;
        0 1 0 0 0 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0];
graphYili = Graph2(numCities);


graphYili.loadAdjacencyMatrix(yili);

% paths = graphYili.printAllPaths(1,1);
paths = graphYili.printAllPaths(2,2);

allPaths = graphYili.m_visitedRoutes
%%

clc
numCities = 6;


% yili = [1 0 0 0; 0 0 0 0; 0 1 0 1; 0 1 0 0];
yili = [1 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 1 0 0;
        0 1 0 0 0 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0];
graphYili = Graph2(numCities);


graphYili.loadAdjacencyMatrix(yili)

% paths1 = graphYili.printAllPaths(1,1)
% allPaths1 = graphYili.m_visitedRoutes
% 
% graphYili.resetObject()
% 
% paths2 = graphYili.printAllPaths(2,1)
% allPaths2 = graphYili.m_visitedRoutes

for i = 1:6
    for j = 1:6
        graphYili.resetObject()
        paths = graphYili.printAllPaths(i,j)
        allPaths = graphYili.m_visitedRoutes
    end
end

% 
% paths = graphYili.printAllPaths(2,1);
% allPaths = graphYili.m_visitedRoutes


%%

oi = [
     3     0     1     0     0;
     1     0     0     0     0;
     0     1     0     1     0;
     0     0     1     0     0;
     0     0     0     0     0;];
 
 graphOi = Graph2(5);
 graphOi.loadAdjacencyMatrix(oi);
 graphOi.printAllPaths(1,1);
 allPaths = graphOi.m_visitedRoutes



%%

% SETUP
clc;
clearvars;


numCities = 6;
yili = [1 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 1 0 0;
        0 1 0 0 0 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0];
    
graphYili = Graph2(numCities);
graphYili.loadAdjacencyMatrix(yili);

% GRAB LIST OF UNVISITED CITIES
pathObject = graphYili.printAllPaths(1,1);
allPathsFromHome = graphYili.m_visitedRoutes;
% 
% Grab list of visited cities from Path-DFS solution:
citiesVisited = [];
for i = 1:numel(allPathsFromHome)
    citiesVisited = [citiesVisited,cell2mat(allPathsFromHome{i})];
end
% Discern unvisited cities:
uniqueCitiesVisited = unique(citiesVisited);
citiesList = 1:1:numCities;
citiesUnvisited = setdiff(citiesList,uniqueCitiesVisited);

% LIST SUBTOURS
subtour = {0}; % Now, subtour has an element, but it won't add constraints
% to the model if it ends up just being 0
for i = citiesUnvisited(1):1:citiesUnvisited(end)
    for j = citiesUnvisited(1):1:citiesUnvisited(end)
        
        
        % Print all subtours as individual cells
        paths = graphYili.printAllPaths(i,j); % i and j represent city indices here
        pathTemp = graphYili.m_visitedRoutes;
        
        % Grab smallest subtour (in terms of number of elements) from
        % pathTemp. The '2' means that the min function looks at the number
        % of columns. We look at the nubmer of columns to see the size
        % because each cell is configured to be a row vector
        [maxSize, maxIdx] = max(cellfun('size', pathTemp,2));
        
        if (isempty(setdiff(cell2mat({1 4 2}),cell2mat(pathTemp{maxIdx}) == true)))
            disp('yonkers')
        end
        
        if (numel(pathTemp{maxIdx}) > numel(subtour{1}))
            subtour = pathTemp(maxIdx);
        end
        
        % If you don't do this stuff, the route nubmer, cnv, and
        % visitedRoutes still stay from old run, screwing things up. Will
        % need to fix later more thoroughly
        graphYili.resetObject(); 

    end
end

% Note that {1,2,3,4,5} is not a good choice. It needs to be an actual
% subtour, not random garbage. Will figure otu lateer
% assert(smallestSubtour~={1,2,3,4,5}, 'no subtours found')

    



%%





%% Notes

% NOTE 1
% Current problem: every time we add an edge (do fcn .addEdge()),
% our member variable self.m_vertexList resets. That's why we can't 
% ever get keys with multiple values

% Specifically, graph.m_vertexList never seems to get filled up. WTF
% SOLUTION: you have to specify if a class will pass stuff by value or by
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



