# Route optimization using BFS, DFS, Uniform Cost Search and A* Search


Given a dataset of major highway segments of the United States (and parts of southern Canada and northern Mexico), 
including highway names, distances, and speed limits; we visualize this as a graph with nodes as towns and highway segments as edges.
We've also prepared a dataset of cities and towns with corresponding latitude-longitude positions.
We implement algorithms
that find good driving directions between pairs of cities given by a user. The program should be
run on the command line like this:
```
./route.py [start-city] [end-city] [routing-algorithm] [cost-function]
```

where:  
     ```[start-city]``` and ```[end-city]``` are the cities we need a route between.  

routing-algorithm is one of:  
    1.```bfs``` uses breadth-first search (which ignores edge weights in the state graph)  
    2.```uniform``` is uniform cost search (the variant of bfs that takes edge weights into consideration)  
    3.```dfs``` uses depth-first search  
    4.```astar``` uses A* search, with a suitable heuristic function  
  
cost-function is one of:  
    1.```segments```: tries to find a route with the fewest number of turns" (i.e. edges of the graph)  
    2.```distance```: tries to find a route with the shortest total distance   
    3.```time```: tries to find the fastest route, for a car that always travels at the speed limit  


The output of the program is human-readable list of directions, including
travel times, distances, intermediate cities, and highway names, similar to what Google Maps or another
site might produce. In addition, the last line of output shows the following machine-readable
output about the route this code has generated:


```
[total-distance-in-miles] [total-time-in-hours] [start-city] [city-1] [city-2] ... [end-city]
```
