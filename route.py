#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# route.py : Build an optimum route between two cities in US
# Augustine Joseph
# Course: CS-B551-Fall2017
#
#
###############################################################################################################
#
# Prof. David Crandall approved 48 hour extension with no penalty for this assignment
#
###############################################################################################################
#
#
"""
###############################################################################################################
#
#					Results
#
#	Start: Atlanta,_Georgia 		End: Dallas,_Texas
###############################################################################################################
#			#		#		    #		    #			#
#	Algorithm	# Cost 		# Time		    # Space 	    # Route		# Travel
#			# Function	# Performance	    # Performance   # Distance(mile)	# Time(hr)
###############################################################################################################
# bfs		    	# distance	# 1.65s		    # 450 nodes     # 1161              # 24.1
#			# segments	# 1.66s		    # 450 nodes     # 1161              # 24.1
#			# time		# 1.68s		    # 450 nodes     # 1161              # 24.1
###############################################################################################################
# dfs			# distance	# 2.97s		    # 3876 nodes    # 41431             # 855.11
#			# segments	# 2.96s		    # 3876 nodes    # 41431             # 855.11
#			# time		# 2.98s		    # 3876 nodes    # 41431             # 855.11
###############################################################################################################
# uniform		# distance	# 1.93s		    # 494 nodes     # 935               # 19.1
#			# segments	# 1.89s		    # 486 nodes     # 1161              # 24.1
#			# time		# 1.88s		    # 480 nodes     # 1027              # 20.27
###############################################################################################################
# Astar			# distance	# 34.00s	    # 90713 nodes   # 798               # 12.74 
#			# segments	# 0.34s		    # 338 nodes     # 991               # 20.48
#			# time		# 0.07s		    # 187 nodes     # 798               # 12.69 
###############################################################################################################

(1) Which search algorithm seems to work best for each routing options? 
    Distance :  Astar gave the best result in terms of smallest distance, but at the cost of higher 
                run time and high memory usage.
    Segments :  Both Astar and BFS gave comparable results while reducing the number of segments. 
                But Astar seems to give better route in terms of route distance and travel time.
    Time     :  Astar gave the best result in terms of lowest computation time. Astar seems to give 
                better results in terms of memory usage, route distance and travel time. 
(2) Which algorithm is fastest in terms of the amount of computation time required by your program, 
    and by how much, according to your experiments? 
    As shown in the table above, Astar algorithm with time as cost function has the lowest computation 
    time(0.07s). This is about 5 times faster than Astar with segments option. Also this is about 25times 
    faster than bfs and uniform cost search algorithms. This search was 42 times faster than DFS.  
(3) Which algorithm requires the least memory, and by how much, according to your experiments? 
    Astar with distance option had the highest memory usage(90713 nodes). DFS has shown high memory usage
    for every cost function option. BFS and uniform cost search seems to have comparable memory usage. 
    Also found that Astar with time option resulted in least memory usage.
(4) Which heuristic function(s) did you use, how good is it, and how might you make it/them better? 
    Using the Haversine formula, we calculated the distance between nodes. This is the shortest distance 
    over the earth’s surface between two points,  ignoring any ups, down and turns that exist in the roadways. 
    So this underestimates the distance and so it is an admissible heuristic. This has been used as heuristic
    function when the distance was the option. However the same heuristic function gave wrong results when 
    segments and time options were used. So the heuristic value has been converted to number of segments, units of 
    time respectively by averaging. This could be an area for improvement. 
    Also the junction nodes were not given the latitude and longitude information. These were approximated to 
    values of next city. This could be improved too. 

Assumptions and Strategies :

1.  The dataset had 0 length and 0 speed segments or edges. All of such data were not considered for route calculations.

2.  Time performance of each algorithm is calculated and reported separately using python time libraries.

3.  Space performance of the algorithms are measured by counting the maximum length of the fringe queue. This 
    measurement is also printed for comparing the results.

4.  Two types of nodes are used for the graph representation of the data: 
    a) Node types of City with certain latitude and longitude, 
    b) Node types of Jct with no latitude and longitude value.

5.  Adjacency Matrix was built to represent this graph data using numpy libraries.

"""



import numpy as np
import sys, math, time
from collections import Counter
import datetime
try:
    import Queue as queue
except ImportError:
    import queue


# Creating Graph
# Nodes are: City, Junctions
# node_type = city or jct

class graph_node:

    def __init__(self, name, node_type, latitude, longitude):

            self.name = name
            self.node_type = ''
            self.latitude = float(latitude)
            self.longitude = float(longitude)

            self.parent = None
            self.child = None


# Creating Edge

class graph_edge:

    def __init__(self, start, end, distance, speed, hwy):

        self.start = start
        self.end = end
        self.distance = int(distance)
        self.speed = int(speed)
        self.hwy = hwy


class fringe_node:

    def __init__(self,cost,node_idx,sol_path):

        self.cost = cost
        self.node_idx = node_idx
        self.sol_path = sol_path


    def __lt__(self, other):

        return self.cost < other.cost



class gen_route:

    def __init__(self,start_node,end_node,rAlgorithm,rOption,node_data,edge_data):

        self.start_node = start_node
        self.end_node = end_node
        self.rAlgorithm = rAlgorithm
        self.rOption = rOption
        self.node_data = node_data
        self.edge_data = edge_data
        self.nodes_dict = {}
        self.graph_adj_matrix = []
        self.Jct_node_list = []
        self.graph_node_list = []
        self.total_speed = 0
        self.total_distance = 0
        self.total_segment = 0
        self.total_time = 0.0
        self.average_speed = 0.0
        self.average_seg_distance = 0
        self.start_nodeIndex = 0
        self.end_nodeIndex = 0
        self.solution_path = ''
        self.end_node_coord = ()

    def build_adjacency_matrix(self):

        for i in range(0, len(self.node_data)):

            self.nodes_dict[self.node_data[i][0]] = i

            # Create Object of class graph_node
            graph_nodeObj = graph_node(self.node_data[i][0], node_type='city', latitude=self.node_data[i][1], longitude=self.node_data[i][2])
            self.graph_node_list.append(graph_nodeObj)

        index = len(self.nodes_dict) +1

        for i in range(0, len(self.edge_data)):

            edge_node1 = self.edge_data[i][0]
            if edge_node1.startswith('Jct') and edge_node1 not in self.Jct_node_list:
                self.nodes_dict[edge_node1] = index
                graph_nodeObj = graph_node(name=edge_node1, node_type='Jct', latitude=0, longitude=0)
                self.Jct_node_list.append(graph_nodeObj)
                index += 1

            edge_node2 = self.edge_data[i][1]
            if edge_node2.startswith('Jct') and edge_node2 not in self.Jct_node_list:
                self.nodes_dict[edge_node2] = index
                graph_nodeObj = graph_node(name=edge_node2, node_type='Jct', latitude=0, longitude=0)
                self.Jct_node_list.append(graph_nodeObj)
                index += 1

        self.graph_node_list = self.graph_node_list + self.Jct_node_list

        self.graph_adj_matrix = [[None]* len(self.graph_node_list) for i in range(len(self.graph_node_list))]

        for i in range(0, len(self.edge_data)):

            edge_node1 = self.edge_data[i][0]
            edge_node2 = self.edge_data[i][1]
            edge_len = self.edge_data[i][2]
            edge_speed = self.edge_data[i][3]
            edge_hwy = self.edge_data[i][4]
            try:
                row_idx = self.nodes_dict[edge_node1]
                column_idx = self.nodes_dict[edge_node2]
                if int(edge_len) == 0 or int(edge_speed) == 0:
                    continue

                self.total_distance += int(edge_len)
                self.total_speed += int(edge_speed)

                time = round(int(edge_len) / float(int(edge_speed)), 2)
                self.total_time += time
                self.total_segment += 1
                graph_edgeObj = graph_edge(edge_node1, edge_node2, edge_len, edge_speed, edge_hwy)
                self.graph_adj_matrix[row_idx][column_idx] = self.graph_adj_matrix[column_idx][row_idx] = graph_edgeObj
            except:
                continue


        self.average_speed = round(self.total_distance / self.total_time, 2)
        self.average_seg_distance = round(self.total_segment / float(self.total_distance), 2)


    def solve(self):
        if self.rAlgorithm == "bfs":
            self.bfs()
        elif self.rAlgorithm == "dfs":
            self.dfs()
        elif self.rAlgorithm == "uniform":
            self.uniform_cost()
        elif self.rAlgorithm == "astar":
            self.astar()

    def h_distance(self, coord1, coord2):
        # Calculate the distance between two [lat,lon] coordnate pairs.
        # Uses Haversine Formula.

        lat1, lon1 = coord1
        lat2, lon2 = coord2

        r = 3959 # radius of earth in miles

        r_lat1=math.radians(lat1)
        r_lat2=math.radians(lat2)

        delta_lat=math.radians(lat2-lat1)
        delta_lon=math.radians(lon2-lon1)

        a=math.sin(delta_lat/2.0)**2+\
           math.cos(r_lat1)*math.cos(r_lat2)*\
           math.sin(delta_lon/2.0)**2

        c=2*math.atan2(math.sqrt(a),math.sqrt(1-a))

        distance = round(r * c, 2)

        return distance




    def bfs(self):
        start = time.time()
        fringe_q = []
        visited_nodes = []
        path_fringe = []
        distance_fringe = []
        time_fringe = []
        fringe_q.append(self.start_nodeIndex)
        fringe_q_max = 1
        path_fringe.append(self.start_node)
        distance_fringe.append(0)
        time_fringe.append(0)

        while self.end_nodeIndex not in fringe_q:
            if fringe_q[0] not in visited_nodes:
                visited_nodes.append(fringe_q[0])
                children_list = self.graph_adj_matrix[fringe_q[0]]
                children_list_idx = []

                for nodeIndex in range(0, len(children_list)):

                    if children_list[nodeIndex] is not None and nodeIndex is not fringe_q[0]:
                        children_list_idx.append(nodeIndex)
                fringe_q.pop(0)
                parent_path_distance = distance_fringe.pop(0)
                parent_path_time = time_fringe.pop(0)
                parent_path = path_fringe.pop(0) + ' '


                for childIndex in range(0, len(children_list_idx)):
                    path_fringe.append(parent_path + self.graph_node_list[children_list_idx[childIndex]].name)
                    distance_fringe.append(parent_path_distance + int(children_list[children_list_idx[childIndex]].distance))
                    time_fringe.append(parent_path_time +
                         round(children_list[children_list_idx[childIndex]].distance / float(
                             children_list[children_list_idx[childIndex]].speed), 2))

                fringe_q = fringe_q + children_list_idx
                if len(fringe_q) > fringe_q_max:
                    fringe_q_max = len(fringe_q)
            else:
                fringe_q.pop(0)
                path_fringe.pop(0)
                distance_fringe.pop(0)
                time_fringe.pop(0)
        idx = fringe_q.index(self.end_nodeIndex)
        self.solution_path = str(distance_fringe[idx]) + ' ' + str(round(time_fringe[idx],2)) + ' ' + str(path_fringe[idx])
        print('Space performance of BFS algorithm: Used a maximum of', str(fringe_q_max), 'nodes in the queue.')
        print('Time taken by BFS algorithm: %0.2fs' % (time.time() - start))
        print("Detail Route :")
        self.print_detail_route(path_fringe[idx])
        print("")
        print(self.solution_path)
        # End of bfs


    def dfs(self):
        start = time.time()
        fringe_q = []
        visited_nodes = []
        path_fringe = []
        distance_fringe = []
        time_fringe = []
        fringe_q.append(self.start_nodeIndex)
        fringe_q_max = 1
        path_fringe.append(self.start_node)
        distance_fringe.append(0)
        time_fringe.append(0)

        while self.end_nodeIndex not in fringe_q:
            if fringe_q[0] not in visited_nodes:
                visited_nodes.append(fringe_q[0])
                children_list = self.graph_adj_matrix[fringe_q[0]]
                children_list_idx = []

                for nodeIndex in range(0, len(children_list)):

                    if children_list[nodeIndex] is not None and nodeIndex is not fringe_q[0]:
                        children_list_idx.append(nodeIndex)
                fringe_q.pop(0)
                parent_path_distance = distance_fringe.pop(0)
                parent_path_time = time_fringe.pop(0)
                parent_path = path_fringe.pop(0) + ' '


                for childIndex in range(0, len(children_list_idx)):
                    path_fringe.insert(childIndex, parent_path + self.graph_node_list[children_list_idx[childIndex]].name)
                    distance_fringe.insert(childIndex, parent_path_distance + int(children_list[children_list_idx[childIndex]].distance))
                    time_fringe.insert(childIndex, parent_path_time +
                         round(children_list[children_list_idx[childIndex]].distance / float(
                             children_list[children_list_idx[childIndex]].speed), 2))

                fringe_q =  children_list_idx + fringe_q
                if len(fringe_q) > fringe_q_max:
                    fringe_q_max = len(fringe_q)
            else:
                fringe_q.pop(0)
                path_fringe.pop(0)
                distance_fringe.pop(0)
                time_fringe.pop(0)
        idx = fringe_q.index(self.end_nodeIndex)
        self.solution_path = str(distance_fringe[idx]) + ' ' + str(round(time_fringe[idx],2)) + ' ' + str(path_fringe[idx])

        print('Space performance of DFS algorithm: Used a maximum of', str(fringe_q_max), 'nodes in the queue.')
        print('Time taken by DFS algorithm: %0.2fs' % (time.time() - start))
        print("Detail Route :")
        self.print_detail_route(path_fringe[idx])
        print("")
        print(self.solution_path)
        # End of dfs



    def uniform_cost(self):
        start = datetime.datetime.now()
        fringe_q = []
        visited_nodes = []
        path_fringe = []
        distance_fringe = []
        time_fringe = []
        fringe_q.append(self.start_nodeIndex)
        fringe_q_max = 1
        path_fringe.append(self.start_node)
        distance_fringe.append(0)
        time_fringe.append(0)

        while( self.end_nodeIndex != fringe_q[0]):
          if( self.end_nodeIndex != fringe_q[0]):
            if fringe_q[0] not in visited_nodes:
                visited_nodes.append(fringe_q[0])
                children_list = self.graph_adj_matrix[fringe_q[0]]
                children_list_idx = []
                children_list_param = []
                for nodeIndex in range(0, len(children_list)):
                    if children_list[nodeIndex] is not None and nodeIndex is not fringe_q[0]:
                        children_list_idx.append(nodeIndex)
                        if self.rOption == 'segments':
                            children_list_param.append(1)
                        elif self.rOption == 'distance':
                            children_list_param.append(children_list[nodeIndex].distance)
                        elif self.rOption == 'time':
                            distance = children_list[nodeIndex].distance
                            speed = children_list[nodeIndex].speed
                            time = round(distance / float(speed), 2)
                            children_list_param.append(time)
                fringe_q.pop(0)
                parent_path_distance = distance_fringe.pop(0)
                parent_path_time = time_fringe.pop(0)
                parent_path = path_fringe.pop(0) + ' '

                sorted_children_list = sorted(range(len(children_list_param)), key=lambda k: children_list_param[k])
                children_list_idx = [children_list_idx[i] for i in sorted_children_list]

                for childIndex in range(0, len(children_list_idx)):
                    path_fringe.append(parent_path + self.graph_node_list[children_list_idx[childIndex]].name)
                    distance_fringe.append(parent_path_distance + int(children_list[children_list_idx[childIndex]].distance))
                    time_fringe.append(parent_path_time +
                         round(children_list[children_list_idx[childIndex]].distance / float(
                             children_list[children_list_idx[childIndex]].speed), 2))

                fringe_q = fringe_q + children_list_idx
                if len(fringe_q) > fringe_q_max:
                    fringe_q_max = len(fringe_q)

            else:

                fringe_q.pop(0)
                path_fringe.pop(0)
                distance_fringe.pop(0)
                time_fringe.pop(0)

        idx = fringe_q.index(self.end_nodeIndex)
        self.solution_path = str(distance_fringe[idx]) + ' ' + str(round(time_fringe[idx], 2)) + ' ' + str(path_fringe[idx])
        delta = datetime.datetime.now() - start
        print('Space performance of Uniform Cost algorithm: Used a maximum of', str(fringe_q_max), 'nodes in the queue.')
        print('Time taken by Uniform Cost algorithm: %0.2fs' % delta.total_seconds())
        print("Detail Route :")
        self.print_detail_route(path_fringe[idx])
        print("")
	
        print(self.solution_path)
        # End of uniform cost


    def get_child_info(self, parent_node, visited_nodes, visited_children_nodes, children_list_idx, children_list_param,
                    children_listAttr, is_Jct, junction_cost):
        children_list = self.graph_adj_matrix[parent_node]
        for childIndex in range(0, len(children_list)):
            # remove None
            if children_list[childIndex] is not None and visited_nodes[childIndex] == 0 and visited_children_nodes[
                childIndex] == 0:

                visited_children_nodes[childIndex] = 1
                name = self.graph_node_list[childIndex].name
                # check if junction
                if self.graph_node_list[childIndex].name.startswith('Jct'):
                    distance = self.graph_adj_matrix[parent_node][childIndex].distance
                    speed = self.graph_adj_matrix[parent_node][childIndex].speed
                    time = round(distance / float(speed), 2)
                    if is_Jct:
                        junction_cost[childIndex] = [
                                                    junction_cost[parent_node][0] + distance,
                                                    junction_cost[parent_node][1] + time,
                                                    junction_cost[parent_node][2] + [name]]
                    else:
                        junction_cost[childIndex] = [distance, time, [name]]
                    self.get_child_info(childIndex, visited_nodes, visited_children_nodes, children_list_idx,
                                     children_list_param,
                                     children_listAttr, True, junction_cost)
                else:
                    children_list_idx.append(childIndex)
                    visit_node_coord = (self.graph_node_list[childIndex].latitude,self.graph_node_list[childIndex].longitude)

                    heuristicCost = self.h_distance(self.end_node_coord, visit_node_coord)
                    distance = children_list[childIndex].distance
                    speed = children_list[childIndex].speed
                    time = round(distance / float(speed), 2)
                    segmentCount = 1
                    if is_Jct:
                        distance += junction_cost[parent_node][0]
                        time += junction_cost[parent_node][1]
                        segmentCount += len(junction_cost[parent_node][2])
                    if self.rOption == 'segments':
                        children_list_param.append(
                            segmentCount + round(heuristicCost * self.average_seg_distance, 2))
                    elif self.rOption == 'distance':
                        children_list_param.append(distance + heuristicCost)
                    elif self.rOption == 'time':
                        children_list_param.append(time + round(heuristicCost / float(self.average_speed), 2))
                    if is_Jct:
                        children_listAttr.append((distance, time, junction_cost[parent_node][2] + [name]))
                    else:
                        children_listAttr.append((distance, time, [name]))

    def astar(self):
        start = time.time()
        self.end_node_coord = (self.graph_node_list[self.end_nodeIndex].latitude,self.graph_node_list[self.end_nodeIndex].longitude)
        fringe_PQ = queue.PriorityQueue()
        visited_nodes = Counter()
        # Parameters are: distance, time, solution path, cost+heuristic
        path_fringe = (0,0, [self.graph_node_list[self.start_nodeIndex].name],0)

        start_node_coord = (
        self.graph_node_list[self.start_nodeIndex].latitude, self.graph_node_list[self.start_nodeIndex].longitude)

        fringe_param = 0 + self.h_distance(self.end_node_coord, start_node_coord)
        fringe_PQ.put(fringe_node(fringe_param, self.start_nodeIndex, path_fringe))
        fringe_pq_max = 1
        fringe_top = fringe_PQ.get()
        while fringe_top.node_idx != self.end_nodeIndex:
            if visited_nodes[fringe_top.node_idx] == 0:
                if self.rOption == 'segments':
                    visited_nodes[fringe_top.node_idx] = 1
                parent_node = fringe_top.node_idx
                children_list_idx = []
                children_list_param = []
                children_listAttr = []
                visited_children_nodes = Counter()
                junction_cost = {}
                # get all children info using parentIndex
                self.get_child_info(parent_node, visited_nodes, visited_children_nodes, children_list_idx,
                                 children_list_param,
                                 children_listAttr, False, junction_cost)

                parent_path_distance = fringe_top.sol_path[0]
                parent_path_time = fringe_top.sol_path[1]
                parent_path = fringe_top.sol_path[2]

                for childIndex in range(0, len(children_list_idx)):
                    if self.rOption == 'segments':
                        p_cost = len(parent_path) + children_list_param[childIndex]
                    elif self.rOption == 'distance':
                        p_cost = parent_path_distance + children_list_param[childIndex]
                    elif self.rOption == 'time':
                        p_cost = parent_path_time + children_list_param[childIndex]
                    p_cost = round(p_cost, 2)
                    path_fringe_node = ( parent_path_distance +
                                         children_listAttr[childIndex][0], parent_path_time +
                                         children_listAttr[childIndex][1],
                                         parent_path + children_listAttr[childIndex][2])


                    fringe_PQ.put(fringe_node(p_cost, children_list_idx[childIndex], path_fringe_node))
                if fringe_PQ.qsize() > fringe_pq_max:
                    fringe_pq_max = fringe_PQ.qsize()
                fringe_top = fringe_PQ.get()
                #
            else:
                fringe_top = fringe_PQ.get()
                
        sol_path=str(fringe_top.sol_path[2]).strip("[").strip("]")
        sol_path=sol_path.replace("'","").replace(", "," ")
        self.solution_path = str(fringe_top.sol_path[0]) + ' ' + str(round(fringe_top.sol_path[1],2)) + ' ' + sol_path

        print('Space performance of Astar algorithm: Used a maximum of', str(fringe_pq_max), 'nodes in the queue.')
        print('Time taken by Astar algorithm: %0.2fs' % (time.time() - start))
        print("Detail Route :")
        self.print_detail_route(sol_path)
        print("")
        print(self.solution_path)
        # End of A*

    def print_detail_route(self, path_fringe):
        path_city=path_fringe.split(" ")
        total_city=len(path_city)
        for i in range(total_city-1) :
            hwy=''
            start_city=path_city[i]
            end_city=path_city[i+1]
            #print(start_city,end_city)
            for j in range(0, len(self.edge_data)):
                if(start_city== self.edge_data[j][0] and end_city==self.edge_data[j][1]) :
                    hwy=self.edge_data[j][4]
                    subroute_distance=float(self.edge_data[j][2])
                    subroute_speed=float(self.edge_data[j][3])
                elif (start_city== self.edge_data[j][1] and end_city==self.edge_data[j][0]) :
                 	hwy=self.edge_data[j][4]
                 	subroute_distance=float(self.edge_data[j][2])
                 	subroute_speed=float(self.edge_data[j][3])
            if(float(subroute_speed)==0 or float(subroute_distance)==0) :
                subroute_distance = 0
                subroute_time = 0
            else:
                subroute_time= float(subroute_distance)/float(subroute_speed)
            print (" Take Highway ", hwy, "From Start City :", start_city, " To End City :", end_city, " Total Distance %.2f :" % subroute_distance , " Estimated Time %.2f" % subroute_time)




edge_data = np.genfromtxt("road-segments.txt", dtype= str, delimiter=' ' )
node_data = np.genfromtxt("city-gps.txt", dtype= str, delimiter=' ' )

args = sys.argv
start_node = args[1]
print("Starting City:", start_node)
end_node = args[2]
print("Ending City:", end_node)
rAlgorithm = args[3]
print("Routing Algorithm:", rAlgorithm)
rOption = args[4]
print("Routing Option:", rOption)

route = gen_route(start_node,end_node,rAlgorithm,rOption,node_data,edge_data)
route.build_adjacency_matrix()
route.start_nodeIndex = route.nodes_dict [start_node]
route.end_nodeIndex = route.nodes_dict [end_node]
route.solve()



