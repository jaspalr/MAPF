from hashlib import new
from ipaddress import collapse_addresses
from sqlite3 import Timestamp
from tarfile import NUL
import time as timer
import heapq
import random
from tkinter.messagebox import NO
from matplotlib.pyplot import flag

from numpy import append, positive
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, pop_node, a_star_mdd
import queue



def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    maxlen = max(len(path1),len(path2)) 
    col = dict()
    for i in range(maxlen):

        if i > 0  and get_location(path1,i-1) == get_location(path2,i) and get_location(path1,i) == get_location(path2,i-1):
           
            col = {'type':'edge','time':i, 'loc':[get_location(path1,i-1),get_location(path1,i) ]}

            return col
        elif get_location(path1,i) == get_location(path2,i):
            col = {'type':'vertex','time':i,'loc': [get_location(path1,i)] }
            return col


    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)):
        for x in range(len(paths)):
            if x > i:
                col = detect_collision(paths[i],paths[x])

                if col != None:
                    
                    collisions.append({'agent1': i, 'agent2' : x, 'loc': col['loc'], 'timestep': col['time']})
                 

    
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    
    list = []
    col = dict()

    if len(collision['loc']) == 1:
        col = {'agent': collision['agent1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        list.append(col)
        col = {'agent': collision['agent2'], 'loc': collision['loc'], 'timestep': collision['timestep'] }
        list.append(col)
        return list
       
    else:
        col = {'agent': collision['agent1'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep']}
        list.append(col)
        col = {'agent': collision['agent2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep']}
        list.append(col)
        return list
       


    pass


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    rand = random.randint(0,1)
    if rand == 0:
        if len(collision['loc']) == 1: 
            return [{'agent': collision['agent1'], 'loc': collision ['loc'], 'timestep': collision['timestep'], 'positive': True},
            {'agent': collision['agent1'], 'loc': collision ['loc'], 'timestep': collision['timestep']}]
        else: 
           
            return [{'agent': collision['agent1'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive': True},
            {'agent': collision['agent1'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep']} ]
             
    else: 
        if len(collision['loc']) == 1: 
            return [{'agent': collision['agent2'], 'loc': collision ['loc'], 'timestep': collision['timestep'], 'positive': True},
            {'agent': collision['agent2'], 'loc': collision ['loc'], 'timestep': collision['timestep']} ]
        else:
            return [{'agent': collision['agent2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive': True},
            {'agent': collision['agent2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep']}]
          
       


           

    pass

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
          
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

def buildMDD(map ,start, goal, herus, i, cons, goalcost):
    
    
    goalnode,lis = a_star_mdd(map,start,goal, herus, i, cons, goalcost)
    root = lis[0]



    
    
    
    
    q = queue.Queue()
    q.put([goalnode])
    mdd = [goalnode]
    mddperlvl = []
    
    for i in range(goalcost + 1):
        mddperlvl.append([]) 
    mddperlvl[goalnode['timestep']].append(goalnode)
    while not q.empty():
        node = q.get()
        node = node[0]
        
        if node['backwardedges'] == None:
            continue
        
        for i in node['backwardedges']:
            
           
           
            if i == goalnode:
                cost = goalcost
            else:
                cost = node['timestep']
            if i['timestep']  == cost - 1:

                if i['children'] == None:
                    i['children'] = [node]
                elif node not in i['children']:
               
                    i['children'].append(node)
                q.put([i])
                mdd.append([i])

  
    
    for i in lis:
        if i['backwardedges'] != None:
            for x in i['backwardedges']:
                if i not in x['children']:
                    i['backwardedges'].remove(x)
                else:
                   if x not in mddperlvl[x['timestep']]:
                    mddperlvl[x['timestep']].append(x)

    
    return root, mddperlvl
    

def comparenodesmutexes(node1,node2,mutexes):
    for mut in mutexes:
        if len(mut) == 2:
            if mut[0]['loc'] == node1['loc'] and mut[0]['timestep'] == node1['timestep']:
                if mut[1]['loc'] == node2['loc'] and mut[1]['timestep'] == node2['timestep']:
                    return True
    return False 

   
def comparemutexes(ls,mutexes):
    istrue = False
    for mut in mutexes:
        if len(mut) == 4:
            if ls[0]['loc'] == mut[0]['loc'] and ls[0]['timestep'] == mut[0]['timestep']:
                if ls[1]['loc'] == mut[1]['loc'] and ls[1]['timestep'] == mut[1]['timestep']:
                    if ls[2]['loc'] == mut[2]['loc'] and ls[2]['timestep'] == mut[2]['timestep']:
                        if ls[3]['loc'] == mut[3]['loc'] and ls[3]['timestep'] == mut[3]['timestep']:
                             return True
     
                   
    
    return False



def findmutex(MDD1, MDD2):
    lst = []
    
   


    if len(MDD1) < len(MDD2):
        length = len(MDD2)
    else:
        length = len(MDD1)

    for i in range(len(MDD2)):
        
        for node in MDD2[i]:
        
            if i < len(MDD1):
                for node2 in MDD1[i]:
                  
                    if node['loc'] == node2['loc']:
                        if not comparenodesmutexes(node,node2, lst):
                                lst.append([node2,node])
                    if node['children'] != None:
                        for child in node['children']:
                            
                            for child2 in node2['children']:
                            
                                if child['loc'] != node['loc'] and child2['loc'] != node2['loc'] and child2['loc'] == node['loc'] and child2['timestep'] - 1 == node['timestep'] and child['loc'] == node2['loc'] and child['timestep'] - 1 == node2['timestep']:
                                    if not comparemutexes([node2,child2,node,child],lst):
                                        lst.append([node,child,node2,child2])
            else:
             
                if(MDD1[-1][0]['loc'] ==  node['loc']):
               
                    lst.append([MDD1[-1][0],node])



            

        


    
   



    return lst
  
    
    


def genmutex(ls):
    mutexes = []
    lst = []
    heapq.heapify(lst)
    i = 0
    for l in ls:
        if len(l) == 2:
            heapq.heappush(lst,(l[0]['timestep'], 1,i))
        else:
            heapq.heappush(lst,(l[0]['timestep'], 2,i))
        i = i + 1




    while(len(lst) != 0):
        _,val,index = heapq.heappop(lst)
        nodes = ls[index] 
        mutexes.append(nodes)
     
     
        if val == 1:
            for nod in nodes[0]['children']:
                for nod2 in nodes[1]['children']:
                    if not comparemutexes([nodes[0],nod,nodes[1],nod2],ls):
                        heapq.heappush(lst,(nod['timestep'], 2, i))
                        ls.append([nodes[0],nod,nodes[1],nod2])
                        i = i +1
        else:
            ispropagatedmutex = True
          
            for node in nodes[1]['backwardedges']:
                for node1 in nodes[3]['backwardedges']:
                    if not comparemutexes([node,nodes[1],node1, nodes[3]],mutexes):
                        ispropagatedmutex = False
            if ispropagatedmutex:
                if not comparenodesmutexes(nodes[1],nodes[3],ls):
                    heapq.heappush(lst,(node1['timestep'], 1,i))

                    ls.append([nodes[1],nodes[3]])
       
            
                    i = i + 1
    return mutexes


def comparenodesmutexes1(node1,node2,mutexes):
    for mut in mutexes:
        if len(mut) == 2:
            if mut[1]['loc'] == node1['loc'] and mut[0]['timestep'] == node1['timestep']:
                if mut[1]['loc'] == node2['loc'] and mut[1]['timestep'] == node2['timestep']:
                    return True
    return False 


#1-PC 2-NC 3-Ac
def CLASSIFY_CONFLICT(MDD1, MDD2, mutexes):
    lst = []
    mincost = 0
    goalnode1 = MDD1[-1][0]
    goalnode2 = MDD2[-1][0]



 
 
  
    for node in MDD2[len(MDD1) - 1]:

       
        
      
        if not comparenodesmutexes(goalnode1,node,mutexes):
            lst.append(node)
           
    if(len(lst) == 0):
        
        return 1

   
    while len(lst) != 0:
        n = lst.pop()
     
        if node['loc'] == goalnode2['loc']:
            return 2
        for child in n['children']:
            
            if child['loc'] == goalnode2['loc']:
              
        
                return 2 
            elif not child['loc'] == goalnode1['loc'] and child['timestep'] >= goalnode1['timestep'] :      
                lst.append(child)
                
  
    return 3 


def GENERATE_CONSTRAINTS_PC(agent1,agent2,lst):
    cons = []
    cons1 = []
    for ls in lst:
        if (len(ls) == 2):
            cons.append({'agent': agent1, 'loc': [ls[0]['loc']], 'timestep': ls[0]['timestep']})
            cons1.append({'agent': agent2, 'loc': [ls[1]['loc']], 'timestep': ls[1]['timestep']})
    return cons,cons1

def GENERATE_CONSTRAINTS_AC(agent1,agent2,lst,goal,time):
    cons = []
    cons.append({'agent': agent1, 'loc': [goal], 'timestep': time})
    cons1 = []
    for ls in lst:
        if(len(ls) == 2):
            if(ls[0]['loc'] == goal and ls[0]['timestep'] == time):
                cons1.append({'agent': agent2, 'loc': [ls[1]['loc']], 'timestep': ls[1]['timestep']})

        
    return cons,cons1






           





        




   






class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, mutex=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """
   
        #disjoint = True #to use disjoint
        
  

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        cols = detect_collisions(root['paths'])
        for col in cols:
            root['collisions'].append(col)
        self.push_node(root)

        global runs 
        runs = 0
        
     
        # Task 3.1: Testing
        #print(root['collisions'])

        
        # Task 3.2: Testing
        #for collision in root['collisions']:
        #  print(collision)
      
      

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        
        while len(self.open_list) > 0:
           
            no = self.pop_node()

  
            
            if len(no['collisions']) == 0:
               
                self.print_results(no)
                paths = no['paths']
                pat = []
                for path in paths:
                    pat.append(path)

                return pat
            col = no['collisions'][0]
            collision = col

        
            

            if (not mutex):
                if disjoint: 
                    constraints = disjoint_splitting(collision)
                else:
                    constraints = standard_splitting(collision)

                for constraint in constraints:
                    newnode = {'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []}
            
                    cons = no['constraints']
                    newnode['constraints'].append(constraint)
                    for con in cons:
                        newnode['constraints'].append(con)
                    flag = False
                    paths = no['paths']
            
                    for path in paths:
                        newnode['paths'].append(path)
                    agent = constraint['agent']
                    path = a_star(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode['constraints'])
                    if path == None:
                        flag = True
                    else:
                        newnode['paths'][agent] = path

                    if disjoint and len(constraint) == 4 and constraint['positive'] and not flag:
                        path = a_star(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode['constraints'])
                        newnode['paths'][agent] = path
                
                        fixpaths = paths_violate_constraint(constraint,newnode['paths'])
                        for fixpath in fixpaths:
                            agent = fixpath
                            path = a_star(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode['constraints'])
                            if path ==  None:
                    
                                flag = True
                                break
                            else:
                                newnode['paths'][agent] = path
            
                    if not flag:
                        cols = detect_collisions(newnode['paths'])
                        newnode['collisions'] = []
                        for col in cols:
                            newnode['collisions'].append(col)
                        newnode['cost'] = get_sum_of_cost(newnode['paths'])
                        self.push_node(newnode)
            else:
                agent = col['agent1']
                agent2 = col['agent2']
                newnode = {'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []}

                newnode1 = {'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []}
                
                cons = no['constraints']
                
                for con in cons:
                    newnode['constraints'].append(con)
                    newnode1['constraints'].append(con)
                    
                paths = no['paths']


            
                for path in paths:
                    newnode['paths'].append(path)
                    newnode1['paths'].append(path)
                #builds mdds for agent1 and agent2, MDDaslistX returns mdd contain in a list the root is MDDaslistX[0] and MDDaslistX[0]['children'] gives all the nodes reachable by the root
                #MDDperlvlAX returns the nodes for given timestep aka the root is in MDDperlvlAX[0] and MDDperlvlAX[1] are nodes reachable by the root

                if len(paths[agent]) > len(paths[agent2]):
                    x = agent2
                    agent2 = agent
                    agent = x
                root1,MDDperlvlA1 = buildMDD(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode['constraints'],len(paths[agent]) -1)
                root2,MDDperlvlA2 = buildMDD(self.my_map,self.starts[agent2], self.goals[agent2], self.heuristics[agent2], agent2, newnode['constraints'],len(paths[agent2]) -1)


             
                

                mutexes = findmutex(MDDperlvlA1,MDDperlvlA2)

         
  

               
                
                mutexe = genmutex(mutexes)

              
                
                
          
               
                
                muts = mutexe
                contype = CLASSIFY_CONFLICT(MDDperlvlA1,MDDperlvlA2,mutexe)
            


               

                if contype != 2:

                    if(len(paths[agent]) > len(paths[agent2])):
                        x = agent2
                        agent2 = agent
                        agent = x
                    cost1 = 0
                    cost2 = 0
                    
                    
                        


                    fdggf = 0
                    type = contype
                    while(type != 2):
                        MDDaslist1,MDDperlvlA1 = buildMDD(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode['constraints'],len(paths[agent]) -1 + cost1 + 1)
                        MDDaslist2,MDDperlvlA2 = buildMDD(self.my_map,self.starts[agent2], self.goals[agent2], self.heuristics[agent2], agent2, newnode['constraints'],len(paths[agent2]) -1 + cost2 +1)
                       
                        fdggf = fdggf + 1
                      
                        

           
                        mutexes = findmutex(MDDperlvlA1,MDDperlvlA2)
                        
                        mutexes = genmutex(mutexes)
                      
                        type  = CLASSIFY_CONFLICT(MDDperlvlA1,MDDperlvlA2,mutexes)
                        
                        if type != 2:
                            cost1 = cost1 + 1
                            cost2 = cost2 + 1
                            muts = mutexes
                            contype = type
                            goal = MDDperlvlA1[-1][0]['timestep']
                            goal1 = MDDperlvlA2[-1][0]['timestep']
                        
                   
                    
                    type = 0
                
                    
                    y = False
                    a1 = agent
                    a2 = agent2
                 
                    while(type != 2):

                        


                        MDDaslist1,MDDperlvlA1 = buildMDD(self.my_map,self.starts[a1], self.goals[a1], self.heuristics[a1], a1, newnode['constraints'],len(paths[a1]) -1 + cost1 +1)
                        MDDaslist2,MDDperlvlA2 = buildMDD(self.my_map,self.starts[a2], self.goals[a2], self.heuristics[a2], a2, newnode['constraints'],len(paths[a2]) -1 + cost2)
                     
                        if MDDperlvlA1[-1][0]['timestep'] > MDDperlvlA2[-1][0]['timestep']:
                           
                            a1 = agent2
                            a2 = agent
                         
                            x = cost1
                            cost1 = cost2
                            cost2 = x
                            MDDaslist1,MDDperlvlA1 = buildMDD(self.my_map,self.starts[a1], self.goals[a1], self.heuristics[a1], a1, newnode['constraints'],len(paths[a1]) -1 + cost1 +1)
                            MDDaslist2,MDDperlvlA2 = buildMDD(self.my_map,self.starts[a2], self.goals[a2], self.heuristics[a2], a2, newnode['constraints'],len(paths[a2]) -1 + cost2)
                           
                            mutexes = findmutex(MDDperlvlA2,MDDperlvlA1)
                            mutexes = genmutex(mutexes)
                            type  = CLASSIFY_CONFLICT(MDDperlvlA2,MDDperlvlA1,mutexes)

                            


                        


                        
                            
                    
                               
                   
                         
                                    
                        else:
                            
                        
                            mutexes = findmutex(MDDperlvlA1,MDDperlvlA2)
                            mutexes = genmutex(mutexes)
                            type  = CLASSIFY_CONFLICT(MDDperlvlA1,MDDperlvlA2,mutexes)
                        if type != 2:
                            cost1 = cost1 + 1
                            muts = mutexes
                            contype = type
                            goal = MDDperlvlA1[-1][0]['timestep'] 
                            goal1 = MDDperlvlA2[-1][0]['timestep']
                            if MDDperlvlA1[-1][0]['timestep'] > MDDperlvlA2[-1][0]['timestep']:
                                agent = a2
                                agent2 = a1
                                goal = MDDperlvlA2[-1][0]['timestep'] 
                
                    
               
                    if(contype == 1):
                        
                        cons,cons1 = GENERATE_CONSTRAINTS_PC(agent,agent2,muts)

                    elif (contype == 3):
                        
                        cons,cons1 = GENERATE_CONSTRAINTS_AC(agent,agent2,muts, self.goals[agent],goal)


        
                    
      
                else:
                    constraints = standard_splitting(collision)

                    cons = [constraints[0]]
                    cons1 = [constraints[1]]
                
                


                
                
                agent = cons[0]['agent']
                for con in cons:
                    newnode['constraints'].append(con)
                  
                path = a_star(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode['constraints'])
                if path != None:
                    newnode['paths'][agent] = path
                    
                    cols = (detect_collisions(newnode['paths']))
                    for col in cols:
                            newnode['collisions'].append(col)
                    newnode['cost'] = get_sum_of_cost(newnode['paths'])
                    self.push_node(newnode)
                    
               
                

             
                if(len(cons1) > 0):
                    agent = cons1[0]['agent']
                    for con in cons1:
                        newnode1['constraints'].append(con)
                    
                    path = a_star(self.my_map,self.starts[agent], self.goals[agent], self.heuristics[agent], agent, newnode1['constraints'])
                    
                    if path != None:
                        newnode1['paths'][agent] = path
                        cols = (detect_collisions(newnode1['paths']))
                        for col in cols:
                                newnode1['collisions'].append(col)
                        newnode1['cost'] = get_sum_of_cost(newnode1['paths'])
                        self.push_node(newnode1)
                        
               
                
                
   
                



             







                


            

                


                  
        print("No solutions")
        return None


    def print_results(self, node):
        print("\n Found a solution! \n")
        #print(node['paths'])
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
