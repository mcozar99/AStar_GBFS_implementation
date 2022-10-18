import pandas as pd
import copy
import time
import sys

#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                              COMMAND PARSING                                                        #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################


BASE_DIR = ''
if len(sys.argv) != 3:
    print(
"""
ERROR: Not enough or too many input arguments.
"""
    )
    sys.exit()
else:
    initial = sys.argv[1]
    goal = sys.argv[2]


#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                         PROBLEM INITIALIZATION                                                      #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################

# We define Nodes for the purpose of the prooblem
class Node:
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def to_dict(self):
        return {
            'state': self.state,
            'parent': self.parent,
            'path_cost': self.path_cost
        }


# Reading data and loading it as to dfs
def load_data(BASE_DIR):
    driving = pd.read_csv(BASE_DIR + 'driving.csv', index_col='STATE')
    straightline = pd.read_csv(BASE_DIR + 'straightline.csv', index_col='STATE')
    return driving, straightline

driving, straightline = load_data(BASE_DIR)
all_states = list(set(driving.index) & set(driving.columns))

if initial not in all_states or goal not in all_states:
    print(
"""
Invalid arguments: START or GOAL state are not part of the graph
""")
    sys.exit()
#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                              COMMON FUNCTIONS                                                       #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################
def calculate_path_cost(path):
    # Given a path, it calculates the total cost of it
    global driving
    cummulative_cost = 0
    i = 1
    while i < len(path):
        cummulative_cost += driving.loc[path[i - 1], path[i]]
        i += 1
    return cummulative_cost


def reconstruct_path(cameFrom, current):
    total_path = []
    total_path.append(current)
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path

def error_ouput(start):
    print('''
Solution path: FAILURE: NO PATH FOUND
Number of states on a path: 0
Path cost: 0
Execution time: %0.4f seconds
    '''%(time.time() - start))

#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                              GREEDY BF SEARCH                                                       #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################

def greedybf_search(initial, goal):
    #   mark start as visited
    #   add start to queue
    visited, queue = [initial], [initial]
    cameFrom = {}

    current_node = ''

    # Start timer
    start = time.time()


    if initial == goal:
        path = [initial]
        execution_time = time.time() - start
        print('''	
Greedy Best First Search:
Solution path: %s
Number of states on a path: %d
Path cost: %d
Straightline cost: %d
Execution time: %0.4f seconds
        ''' % (
        ', '.join(path), 1, 0, straightline.loc[initial, goal], execution_time))
        return ', '.join(path), 1, 0, execution_time
    while queue != []:
        # Order queue according to heuristics
        costs = []
        for state in queue:
          costs.append(straightline.loc[state, goal])
        df = pd.DataFrame({'state': queue, 'cost': costs})
        queue = df.sort_values(by='cost').state.to_list()

        # current_node ← vertex of queue with min distance to target and remove current from queue
        previous_node = current_node
        current_node = queue[0]
        # Annotate the path
        cameFrom[current_node] = previous_node

        queue.remove(current_node)
        # extract neighbors
        neighbors = driving.loc[current_node, :]
        neighbors = neighbors[neighbors > 0].sort_values()
        for neighbor in neighbors.index:
            if neighbor == goal:
                cameFrom[goal] = current_node
                path = reconstruct_path(cameFrom, goal)
                path.pop(0)
                # Generation of outputs neccesary for the last step
                execution_time = time.time() - start
                path_cost = calculate_path_cost(path)
                print('''	
Greedy Best First Search:
Solution path: %s
Number of states on a path: %d
Path cost: %d
Straightline cost: %d
Execution time: %0.4f seconds
                      ''' % (', '.join(path), len(path), path_cost, straightline.loc[initial, goal], execution_time))
                return ', '.join(path), len(path), path_cost, execution_time
            else:
                if neighbor in cameFrom.keys():
                    continue
                queue.append(neighbor)
                visited.append(neighbor)

    print('Greedy Best First Search:')
    error_ouput(start)

#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                                   A* SEARCH                                                         #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################

def a_search(start, goal):
    """
    reference: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
    """
    # The set of discovered nodes that may need to be (re-)expanded.
    # Initially, only the start node is known
    # This is usually implemented as a min-heap or priority queue rather than a hash-set
    openSet = [start]

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path to n currently known
    cameFrom = {}

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known
    gScore = {}
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    # how cheap a path could be from start to finish if it goes through n.
    fScore = {}
    fScore[start] = straightline.loc[start, goal]

    start = time.time()

    if initial == goal:
        path = [initial]
        execution_time = time.time() - start
        print('''	
A* Search:
Solution path: %s
Number of states on a path: %d
Path cost: %d
Straightline cost: %d
Execution time: %0.4f seconds
            ''' % (
            ', '.join(path), 1, 0, straightline.loc[initial, goal], execution_time))
        return ', '.join(path), 1, 0, execution_time

    while openSet != []:
        # Order the openSet
        costs = []
        for state in openSet:
          costs.append(fScore[state])
        df = pd.DataFrame({'state': openSet, 'cost': costs})
        openSet = df.sort_values(by='cost').state.to_list()

        current = openSet[0]  # the node in openSet having the lowest fScore[] value
        if current == goal:
          path = reconstruct_path(cameFrom, current)
          execution_time = time.time() - start
          print('''
A* Search:
Solution path: %s
Number of states on a path: %d
Path cost: %d
Straightline cost: %d
Execution time: %0.4f seconds
''' % (
          ', '.join(path), len(path), calculate_path_cost(path), straightline.loc[initial, goal],
          execution_time))
          return ', '.join(path), len(path), calculate_path_cost(path), execution_time

        openSet.remove(current)

        neighbors = driving.loc[current, :]
        neighbors = neighbors[neighbors > 0].sort_values()
        for neighbor in neighbors.index:
          # d(current,neighbor) is the weight of the edge from current to neighbor
          # tentative_gScore is the distance from start to the neighbor through current
          tentative_gScore = gScore[current] + driving.loc[current, neighbor]

          if neighbor in gScore.keys():
            if tentative_gScore < gScore[neighbor]:
              # This path to neighbor is better than any previous one. Record it!
              cameFrom[neighbor] = current
              gScore[neighbor] = tentative_gScore
              fScore[neighbor] = tentative_gScore + straightline.loc[current, neighbor]
              if neighbor not in openSet:
                openSet.append(neighbor)
          else:
            cameFrom[neighbor] = current
            gScore[neighbor] = tentative_gScore
            fScore[neighbor] = tentative_gScore + straightline.loc[current, neighbor]
            openSet.append(neighbor)
    print('A* Search:')
    error_ouput(start)




#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                                  TESTING                                                            #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################

def random_test(num_tests):
    states = straightline.index.to_list()
    import random
    initials, goals, paths_a, paths_greed, costs_a, costs_greed, lens_greed, lens_a, times_greed, times_a = [], [], [], [], [], [], [], [], [], []
    for i in range(num_tests):
        initial = random.choice(states)
        initials.append(initial)
        goal = random.choice(states)
        goals.append(goal)
        print(initial, goal)
        path_greed, len_greed, cost_greed, time_greed = greedybf_search(initial, goal)
        lens_greed.append(len_greed)
        paths_greed.append(path_greed)
        costs_greed.append(cost_greed)
        times_greed.append(time_greed)
        path_a, len_a, cost_a, time_a = a_search(initial, goal)
        paths_a.append(path_a)
        costs_a.append(cost_a)
        lens_a.append(len_a)
        times_a.append(time_a)
        time.sleep(0.5)
    df = pd.DataFrame({'initial': initials, 'goal': goals, 'cost_a': costs_a,
                       'cost_greed': costs_greed, 'len_a':lens_a, 'len_greed': lens_greed,
                       'path_a': paths_a, 'path_greed':paths_greed, 'time_a': times_a,
                       'time_greed': times_greed})
    df.to_excel('random_test.xlsx', index=False)

def directed_test(init_states, goal_states):
    initials, goals, paths_a, paths_greed, costs_a, costs_greed, lens_greed, lens_a, times_greed, times_a = [], [], [], [], [], [], [], [], [], []
    for i in range(len(init_states)):
        initial = init_states[i]
        initials.append(initial)
        goal = goal_states[i]
        goals.append(goal)
        path_greed, len_greed, cost_greed, time_greed = greedybf_search(initial, goal)
        lens_greed.append(len_greed)
        paths_greed.append(path_greed)
        costs_greed.append(cost_greed)
        times_greed.append(time_greed)
        path_a, len_a, cost_a, time_a = a_search(initial, goal)
        paths_a.append(path_a)
        costs_a.append(cost_a)
        lens_a.append(len_a)
        times_a.append(time_a)
    df = pd.DataFrame({'initial': initials, 'goal': goals, 'cost_a': costs_a,
                       'cost_greed': costs_greed, 'len_a': lens_a, 'len_greed': lens_greed,
                       'path_a': paths_a, 'path_greed': paths_greed, 'time_a': times_a,
                       'time_greed': times_greed})
    df.to_excel('directed_test.xlsx', index=False)

#######################################################################################################################
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#                                                  OUTPUT                                                             #
#                                                                                                                     #
#                                                                                                                     #
#                                                                                                                     #
#######################################################################################################################
print(
"""
Cozar Tramblin, Miguel, A20522001 solution:
Initial state: %s
Goal state: %s
"""%(initial, goal)
)

greedybf_search(initial, goal)
a_search(initial, goal)


# TEST FOR REPORT

#initials = ['CO','CO','CO','CO','CO','CO','CO','CO','CO','CO']
#goals =    ['ME','ME','ME','ME','ME','ME','ME','ME','ME','ME' ]

#directed_test(initials, goals)

#random_test(200)
