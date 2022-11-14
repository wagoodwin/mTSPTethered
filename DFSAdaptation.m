

clc


radda = Graph(4);

% note: with this notation, radda is actually passed as an argument 
% ALONG with 1 and 2. So it's really like addEdge(radda,1,2).
radda.addEdge(1, 2)
radda.addEdge(1, 3)
radda.addEdge(1, 4)
radda.addEdge(3, 1)
radda.addEdge(3, 2)
radda.addEdge(2, 4)

radda.printAllPaths(3,4)

%%

allPaths = radda.m_currentVertexPaths



%% Notes

% NOTE 1
% Current problem: every time we add an edge (do fcn .addEdge()),
% our member variable self.m_vertexList resets. That's why we can't 
% ever get keys with multiple values

% Specifically, radda.m_vertexList never seems to get filled up. WTF
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



