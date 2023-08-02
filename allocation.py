import deps.matching as matching
import numpy as np

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.agent import arbi_agent_executor
from arbi_agent.model import generalized_list_factory as GLFactory

import time
import deps.printInColor as pic

import conversions as c
import robotPlan_class as rc

from log.setup import logger

def allocationCore(robots,goals,arbiAgent,arbiMAPF):
    logger.info("robots={}".format(robots))
    logger.info("goals={}".format(goals))
    
    #create and fill cost matrix
    cost_mat = generateCostMatrix(robots,goals,arbiAgent,arbiMAPF)
    #fill missing rows/cols to make it square
    cost_mat = c.makeSqMat(cost_mat)
    #assignment, cost = matching.matching(cost_mat, numWays)
    assignment, cost = matching.matching(cost_mat)
    allocMat = assignment.astype("int")

    #remove allocation with super large cost
    c_rows, c_cols = cost_mat.shape
    for i in range(c_rows):
        for j in range(c_cols):
            if(cost_mat[i,j] > (c.superLargeCost-1)):
                allocMat[i,j] = int(0)

    #print("***RESULT (numWays = %d)***\n" %numWays)
    # print("***RESULT***\n")
    # print("The cost matrix")
    # print(cost_mat)
    # print("\nThe optimal allocation")
    # print(allocMat)
    # print("\nThe cost sum: %f" %cost)
    logger.info("***RESULT***\n")
    logger.info("The cost matrix")
    logger.info(cost_mat)
    logger.info("\nThe optimal allocation")
    logger.info(allocMat)
    logger.info("\nThe cost sum: %f" %cost)
    
    # consider not allocated agents
    #iterate matrix to extract allocation result
    #result = name1:node1,node2, ... ,noden;name2:node1,node2, ... ,noden
    allocRobotPlans = []
    for row in range(len(robots)):
        if sum(allocMat[row]) == 0:
            robots[row].goal = robots[row].start
            allocRobotPlans.append(robots[row])
        else:
            for col in range(len(goals)):
                if(allocMat[row][col] == 1):
                    robots[row].goal = goals[col]
                    allocRobotPlans.append(robots[row])
    
    return allocRobotPlans

#response in arbi Gl format
def planMultiAgentReqest(robotPlans, arbi, arbiMAPF):
    msgByRobot = []
    for r in robotPlans:
        if(len(r.goal) > 0):
            singleMsg = (str(r.name) + str(c.robot_path_delim) + str(r.start) + str(c.robot_path_delim) + str(r.goal))
            msgByRobot.append(singleMsg)
    reqMsg = c.robot_robot_delim.join(msgByRobot)
    print(reqMsg)
    #convert to arbi msg
    arbiMsg = c.msg2arbi_req(reqMsg)
    print(arbiMsg)
    reqStartTime = time.time()
    #res = arbi.request(arbiMAPF,reqMsg)
    res = arbi.request(arbiMAPF,arbiMsg)
    reqEndTime = time.time()
    # pic.printC("Handling request took " + str(reqEndTime - reqStartTime) + " seconds", 'warning')
    logger.warning("Handling request took " + str(reqEndTime - reqStartTime) + " seconds")

    #return in gl format
    return res
    #return res


def generateCostMatrix(robotPlans,goals,arbi,arbiMAPF):
    costMat = np.ones((len(robotPlans), len(goals)))
    #iterate to fill matrix
    #for each robot
    serializedCosts = []
    for r in robotPlans:
        for g in goals:
            # name1,start1,goal1;name2,start2,goal2, ...
            #msg = (r.name + "," + r.start + "," + g)
            #send through arbi
            #reqStartTime = time.time()
            #pathMsg = arbi.query(arbiMAPF,msg)
            pathMsg_gl = planMultiAgentReqest([rc.robotPlan(r.name,r.start,g)],arbi,arbiMAPF)
            pathMsg = c.arbi2msg_res(pathMsg_gl)
            #reqEndTime = time.time()
            #print("Request took " + str(reqEndTime - reqStartTime) + " seconds")
            #expect to have result for only one robot
            planResultDict = {}
            if(len(pathMsg)>0):
                planResultDict = c.responsToDict(pathMsg)
            #fill cost matrix
            #if robot name is matched
            if(r.name in planResultDict):
                #if plan was a success
                if(planResultDict[r.name][0] != 'failed'):
                    serializedCosts.append(len(planResultDict[r.name]))
                else:
                    #path plan failed. apply super large cost
                    serializedCosts.append(c.superLargeCost)
            else:
                #path plan failed. apply super large cost
                    serializedCosts.append(c.superLargeCost)
            """
            pathMsg_sp = pathMsg.split(',')
            #if robot name is matched
            path_list = []
            if(pathMsg_sp[0] == r.name):
                if(pathMsg_sp != 'failed'):
                    path_list = pathMsg_sp[1].split('-')
                    serializedCosts.append(len(path_list))
                else:
                    #path plan failed. apply super large cost
                    serializedCosts.append(superLargeCost)
            """
            
    #fill matrix out of serialized costs
    #by row vectors
    for ind in range(0,len(serializedCosts)):
        row = ind // len(goals)
        col = ind % len(goals)
        costMat[row,col] = serializedCosts[ind]

    return costMat
