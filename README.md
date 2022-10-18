# A* and Greedy Best-First Search Algortihms implementation

This work was done at Illinois tech introductory course to Artificial Intelligence as a programming assignment. To execute the program just call:

```shell
python main.py [initial state] [goal state]
```

The data provided is the map of the US states and the driving and straight line distances between all of them. To change the state map, you simpli have to change the csv files into the ones you want and change them in the function loading the data.

Output examples:

```shell
python .\main.py IL CA

>>>
Cozar Tramblin, Miguel, A20522001 solution:
Initial state: IL
Goal state: CA


Greedy Best First Search:
Solution path: IL, IA, SD, MT, ID, NV, CA
Number of states on a path: 7
Path cost: 2628
Straightline cost: 1699
Execution time: 0.0070 seconds


A* Search:
Solution path: IL, IA, NE, WY, UT, NV, CA
Number of states on a path: 7
Path cost: 2083
Straightline cost: 1699
Execution time: 0.0405 seconds
```

Code made by @mcozar99, with Wikipedia used as source of information.