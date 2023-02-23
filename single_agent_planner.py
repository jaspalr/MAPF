from calendar import c
import heapq

from numpy import true_divide

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0),(0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraintslist = dict()
   
    for constraint in constraints:
        
        if len(constraint) == 3:
            
            time = constraint['timestep']
            if constraint['agent'] == agent:
                if time in constraintslist.keys():
                    loc = constraint['loc']
                    constraintslist[time].append(loc)
                else:
                    loc = constraint['loc']
                    constraintslist[time] = [loc]
   
        else:

            if constraint['timestep'] in constraintslist.keys():
                if constraint['agent'] == agent:
                    constraintslist[constraint['timestep']].append([constraint['loc'],[constraint['positive']]])
                else:
                    constraintslist[constraint['timestep']].append([constraint['loc'],[False]])
            else:
                if constraint['agent'] == agent:
                    constraintslist[constraint['timestep']] = [[constraint['loc'],[constraint['positive']]]]
                else:
                    constraintslist[constraint['timestep']] = [[constraint['loc'],[False]]]

    return constraintslist


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    for time in constraint_table:
        if next_time == time:
            constraints = constraint_table[time]
            for constraint in constraints:
                loc = constraint  
                if len(loc) == 1:

                    if loc == [next_loc]:
                        return True 
    
                elif len(loc) == 2 and not isinstance(loc[1], list):
                    if loc == [curr_loc,next_loc]:
                        return True
    
                elif len(loc) >= 2 and isinstance(loc[1], list):
                    if len(loc[0]) == 1:
                        if loc[1][0] == False and loc[0][0] == next_loc:
                            return True
                        elif  loc[1][0] == True and loc[0] != [next_loc]:
                            return True
                    else:
                        if loc[1][0] == False and (loc[0][0] == curr_loc or loc[0][1] == next_loc \
                        or loc[0] == [next_loc,curr_loc]):
                            return True
                        elif  loc[1][0] == True and loc[0] != [curr_loc,next_loc]:
                           return True


                            
                            
                       
        elif time < 0 and next_time >= -time: ##checks if time is negative and -time is equal or greater than next_time
        
            locs = constraint_table[time]
            for loc in locs:
               if len(loc) == 1 and loc == [next_loc]:
                  return True
                
     
    
        
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    count = 0
    for i in range(len(my_map)):
        for x in range(len(my_map[i])):
            if my_map[i][x] == False:
                count = count + 1
    count = count * count 
    constraintslist= build_constraint_table(constraints, agent)
   
 
    
    goal = [goal_loc]
    
    for time, loc in constraintslist.items():
        
     
        
      
        if len(loc[0]) >= 2 and (loc[0][1] == [True] or  loc[0][1] == [False]):
            for l in loc:               
                if len(l) ==2:  
                    if not l[1][0] and goal == l[0]:
                        earliest_goal_timestep = max(earliest_goal_timestep,time)
                elif goal == l:
                    earliest_goal_timestep = max(earliest_goal_timestep,time)

        elif goal in loc:
            earliest_goal_timestep = max(earliest_goal_timestep,time)

    

    inc = 0

   
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'],root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr)
        
        for dir in range(5):
            
            child_loc = move(curr['loc'], dir)
            inc = inc + 1

            if child_loc[0] < 0 or child_loc[1] < 0:
                   continue
            if(len(my_map) <= child_loc[0]):
                continue
            if len(my_map[child_loc[0]]) <= child_loc[1]:
                continue
            
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if inc > count:
                return None
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep' : curr['timestep'] + 1}
         
            if not is_constrained(curr['loc'],child_loc,child['timestep'],constraintslist):
               
                
                if (child['loc'], child['timestep']) in closed_list:
       
                    existing_node = closed_list[(child['loc'],child['timestep'])]
       
                    if compare_nodes(child, existing_node):
                       
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

    return None  # Failed to find solutions

def a_star_mdd(my_map, start_loc, goal_loc, h_values, agent, constraints, goalcost):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    count = 0
    
    for i in range(len(my_map)):
        for x in range(len(my_map[i])):
            if my_map[i][x] == False:
                count = count + 1
    count = count * count * count * count
    constraintslist= build_constraint_table(constraints, agent)

    lis = []
 
    
    goal = [goal_loc]
    for time, loc in constraintslist.items():

     
        
      
        if len(loc[0]) >= 2 and (loc[0][1] == [True] or  loc[0][1] == [False]):
            for l in loc:               
                if len(l) ==2:  
                    if not l[1][0] and goal == l[0]:
                        earliest_goal_timestep = max(earliest_goal_timestep,time)
                elif goal == l:
                    earliest_goal_timestep = max(earliest_goal_timestep,time)

        elif goal in loc:
            earliest_goal_timestep = max(earliest_goal_timestep,time)
        
    

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0, 'children': [], 'backwardedges': []}
    push_node(open_list, root)
    closed_list[(root['loc'],root['timestep'])] = root
    inc = 0
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep and curr['timestep'] == goalcost:
            goalnode = curr
        
        if curr['g_val'] + curr['h_val'] > goalcost:
            
            return goalnode,lis
        lis.append(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            inc = inc + 1


            if child_loc[0] < 0 or child_loc[1] < 0:
                   continue
            if(len(my_map) <= child_loc[0]):
                continue
            if len(my_map[child_loc[0]]) <= child_loc[1]:
                continue
            
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if inc > count:
                return None
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep' : curr['timestep'] + 1,
                    'children': [],
                    'backwardedges': []}
         
            if not is_constrained(curr['loc'],child_loc,child['timestep'],constraintslist):
               
                
                if (child['loc'], child['timestep']) in closed_list:
       
                    existing_node = closed_list[(child['loc'],child['timestep'])]
       
                    if compare_nodes(child, existing_node):

                        child['backwardedges'] = existing_node['backwardedges'].copy()
                        if goalcost >= child['g_val'] + child['h_val']:
                            if child['backwardedges'] == None:
                                child['backwardedges'] = [child['parent']]
                            else:
                                child['backwardedges'].append(curr)
                        existing_node['backwardedges'] = None
                        existing_node['parent'] = None 


                       
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                    else:
                        if existing_node['backwardedges'] == None:
                            if goalcost >= child['g_val'] + child['h_val']:
                                existing_node['backwardedges'] = [child['parent']]
                        else:
                            existing_node['backwardedges'].append(child['parent'])
                        child['parent'] = None

                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    if goalcost >= child['g_val'] + child['h_val']:
                        if child['backwardedges'] == None:

                            child['backwardedges'] = [child['parent']]
                        else:
                            child['backwardedges'].append(child['parent'])

                    push_node(open_list, child)

    return [],[]  # Failed to find solutions
