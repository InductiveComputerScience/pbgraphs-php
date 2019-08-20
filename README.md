# pbgraphs-php
pbGraphs-php is a Graph Library in PHP

## Requirements
PHP 5+.

## Dependencies
There are no dependencies.

## Getting Started
It is easy to get started. Simply include the sources in your project. Check out test.php in the source directory for an example of how to use it.

## Avaialble in 12 different programming languages
pbGraphs-java has been implemented with [progsbase](https://www.progsbase.com), so the library is available for 12 different programming languages.

Try [all functions online](https://repo.progsbase.com/repoviewer/no.inductive.libraries/DirectedGraphs/0.1.14/) and see the implementations in different languages.

## Functions

### Graph Equality
```
function DirectedGraphsEqual($a, $b)
```

Returns true of the two directed graphs are equal.

### Graph Components
```
function GetGraphComponents($g, $componentMembership)
```

Places the list of strongly connected components in componentMembership. Returns false if graphs does not quality as undirected.

### Topological Sort
```
function TopologicalSort($g, $list)
```

Places the topological sort of the graph in `list`.

### Searches

* Depth-first Search
```
function DepthFirstSearch($g, $start, $list)
```

Places the depth-first sort ordering of the graph in `list`.

* Breadth-first Search
```
function BreadthFirstSearch($g, $start, $list)
```

Places the breadth-first sort ordering of the graph in `list`.

### Shortest Paths

* Dijkstra's Algorithm
```
function DijkstrasAlgorithm($g, $src, $dist, $distSet, $prev)
```

Performs Dijkstra's algorithm on the graph `g` from `src`. Whether nodes are reachable is placed in `distSet`, the shortest distances in `dist`, and the previous node in the shortest paths in `prev`.

* Bellman-Ford Algorithm
```
function BellmanFordAlgorithm($g, $src, $dist, $distSet, $prev)
```

Performs the Bellman-Ford algorithm on the graph `g` from `src`. Whether nodes are reachable is placed in `distSet`, the shortest distances in `dist`, and the previous node in the shortest paths in `prev`.


* Floyd-Warshall Algorithm
```
function FloydWarshallAlgorithm($g, $distances)
```

Performs the Floyd-Warshall algorithm on the graph `g`. The shortest distances between each pair of nodes are placed in `distances`.

### Minimum Spanning Trees

* Prim's Algorithm
```
function PrimsAlgorithm($g, $forest)
```

Performs the Prim's algorithm on the graph `g`. All minimum spanning trees of the graph are placed in `forest`. Returns false if graphs does not quality as undirected.


* Kruskal's Algorithm
```
function KruskalsAlgorithm($g, $forest)
```

Performs the Kruskal's algorithm on the graph `g`. All minimum spanning trees of the graph are placed in `forest`. Returns false if graphs does not quality as undirected.

### Cycles

* Cycle Detection
```
function DirectedGraphContainsCycleDFS($g)
```

Return true if there are cycles in the graph `g`.

* Cycle Counting
```
function DirectedGraphCountCyclesDFS($g)
```

Return the number of cycles in the graph `g`.

* Get All Cyles
```
function &DirectedGraphGetCyclesDFS($g)
```

Returns the list of cycles in the graph `g`.