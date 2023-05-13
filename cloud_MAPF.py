"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
Modifed: Ahn, Jeeho
"""
#from os import getresgid, path
import sys
import copy
#sys.path.insert(0, '../')
import argparse
from typing import AsyncGenerator
import yaml
#Dr. Oh Map Parse Tool
from map_parse import MapMOS as mapParser
import time
import deps.mapElements as mapElements

import deps.planningTools as pt
import deps.printInColor as pic
from deps.cbs import CBS
from deps.cbs3 import CBS3
#import handler_tools as ht

robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'

args = {"param":"yaml/input.yaml","output":"yaml/output.yaml"}
MAP_CLOUD_PATH = "map_parse/map_cloud.txt"

def msg2agentList(msg):
    # name1,start1,goal1;name2,start2,goal2, ...
    agentsList = []
    byRobots = msg.split(robot_robot_delim)
    for r in byRobots:
        elems = r.split(robot_path_delim)
        #convert node name to map index coord.
        start_xy = pt.graph2grid(elems[1],vertices_with_name)
        goal_xy = pt.graph2grid(elems[2],vertices_with_name)
        agentsList.append({'start':[start_xy[0],start_xy[1]], 'goal':[goal_xy[0],goal_xy[1]], 'name':elems[0]})

    return agentsList


def planning_loop(agents_in,print_result=True):
    #wait and get agents from server, do the job, then repeat
    #assume agents info(agents_in) is received    
    loop_start = time.time()

    #initialize env
    env = pt.Environment(mapElems.dimension, agents_in, mapElems.obstacles, mapElems.vertices_with_name, mapElems.edges_dict)
    
    # Searching
    # cbs = CBS(env)
    cbs = CBS3(env)
    start = time.time()
    solution = cbs.search(print_=print_result)
    end = time.time()
    if(print_result == True):
        print(end - start)
    if not solution:
        pic.printC(" Solution not found",'warning')
        return -1

    # # Write to output file
    # with open(args['output'], 'r') as output_yaml:
    #     try:
    #         output = yaml.load(output_yaml, Loader=yaml.FullLoader)
    #     except yaml.YAMLError as exc:
    #         print(exc)

    # output["schedule"] = solution
    # output["cost"] = env.compute_solution_cost(solution)
    # with open(args['output'], 'w') as output_yaml:
    #     yaml.safe_dump(output, output_yaml)

    #Jeeho Edit
    #convert resulting path to node names
    sol_in_node_name = env.solution2NodeNames(solution)
    if(print_result==True):
        print(sol_in_node_name)

    #send through ARBI
    loop_end = time.time()
    if(print_result==True):
        pic.printC("Planning Loop took: " + str(loop_end - loop_start) + " seconds",'green')
    return sol_in_node_name
    #repeating ends here

def planMultiAgentReqest(test_robots, print_result=True):
    #Initialize Arbi Client Agent
    #start an agent

    # Read from input file
    with open(args['param'], 'r') as arg:
        try:
            param = yaml.load(arg, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # get param
    dimension = param["map"]["dimensions"]
    #obstacles = param["map"]["obstacles"]
    vertices_yaml = param["map"]["vertices"] #list
    agents = param['agents']

    vertices = []
    global vertices_with_name
    vertices_with_name = [] #list of tuple
    for item in vertices_yaml:
        vertices.append((item[0],item[1]))
        vertices_with_name.append(((item[0],item[1]),item[2],item[3]))

    # assume each target is not an obstacle node
    obstacles = [] 
    for x in range(dimension[0]):
        for y in range(dimension[1]):
            if((x,y) not in vertices):
                obstacles.append((x,y))    

    #initialize Map Elements data
    #Environment should be re-initialized with modified agents
    global mapElems
    edges_dict = mapParser.MapMOS(MAP_CLOUD_PATH).Edge
    mapElems = mapElements.mapElements(dimension,obstacles,vertices_with_name,edges_dict)

    #add agents as below
    #agents_in.append({'start':[0,0], 'goal':[1,1], 'name':'agent_new'})
    agents_in = []

    msg_list = []
    for r in test_robots:
        msg_list.append(r[0] + robot_path_delim + r[1] + robot_path_delim + r[2])

    msg = robot_robot_delim.join(msg_list)
    agents_in = msg2agentList(msg)
    plan = planning_loop(agents_in, print_result)
    #test run end

    return plan

def main():
    # case1
    a1 = ["AMR_LIFT1", "140", "118"]
    # a2 = ["AMR_LIFT2", "107", "104"]
    # a3 = ["AMR_LIFT3", "150", "116"]
    # a4 = ["AMR_LIFT4", "116", "115"]

    test_robots = []
    test_robots.append(a1)    
    # test_robots.append(a2)
    # test_robots.append(a3)
    # test_robots.append(a4)
    plan = planMultiAgentReqest(test_robots)
    print(plan)

if __name__ == "__main__":
    main()