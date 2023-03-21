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

TPL_address = "agent://www.arbi.com/TaskPolicyLearner"

def allocationCore(robots, goals, arbiAgent, arbiMAPF):
    # create and fill cost matrix
    cost_mat = generateCostMatrix(robots, goals, arbiAgent, arbiMAPF)


    # create and fill safety, efficiency matrix
    tpl_res = req_TPL(robots,goals,arbiAgent,arbiMAPF)
    s_mat, e_mat = generateSEMatrix(tpl_res)

    s_mat = c.makeSqMat(s_mat)
    e_mat = c.makeSqMat(e_mat)

    se_mat = s_mat + e_mat
    se_mat_to_cost = se_mat * -1

    # fill missing rows/cols to make it square
    cost_mat = c.makeSqMat(cost_mat)
    cost_mat = cost_mat + se_mat_to_cost
    # assignment, cost = matching.matching(cost_mat, numWays)
    assignment, cost = matching.matching(cost_mat)
    allocMat = assignment.astype("int")

    # remove allocation with super large cost
    c_rows, c_cols = cost_mat.shape
    for i in range(c_rows):
        for j in range(c_cols):
            if (cost_mat[i, j] > (c.superLargeCost - 1)):
                allocMat[i, j] = int(0)

    # print("***RESULT (numWays = %d)***\n" %numWays)
    print("***RESULT***\n")
    print("The cost matrix")
    print(cost_mat)
    print("Safety matrix")
    print(s_mat)
    print("Efficiency matrix")
    print(e_mat)
    print("\nThe optimal allocation")
    print(allocMat)
    print("\nThe cost sum: %f" % cost)

    # consider not allocated agents
    # iterate matrix to extract allocation result
    # result = name1:node1,node2, ... ,noden;name2:node1,node2, ... ,noden
    allocRobotPlans = []
    for row in range(len(robots)):
        if sum(allocMat[row]) == 0:
            robots[row].goal = str(robots[row].start)
            allocRobotPlans.append(robots[row])
        else:
            for col in range(len(goals)):
                if (allocMat[row][col] == 1):
                    robots[row].goal = str(goals[col])
                    allocRobotPlans.append(robots[row])

    return allocRobotPlans


# response in arbi Gl format
def planMultiAgentReqest(robotPlans, arbi, arbiMAPF):
    msgByRobot = []
    for r in robotPlans:
        if (len(r.goal) > 0):
            singleMsg = (str(r.name) + str(c.robot_path_delim) + str(r.start) + str(c.robot_path_delim) + str(r.goal))
            msgByRobot.append(singleMsg)
    reqMsg = c.robot_robot_delim.join(msgByRobot)
    # convert to arbi msg
    arbiMsg = c.msg2arbi_req(reqMsg)
    reqStartTime = time.time()
    # res = arbi.request(arbiMAPF,reqMsg)
    res = arbi.request(arbiMAPF, arbiMsg)
    reqEndTime = time.time()
    pic.printC("Handling request took " + str(reqEndTime - reqStartTime) + " seconds", 'warning')

    # return in gl format
    return res
    # return res


# request to TPL
def req_TPL(robots, goal, arbi, arbiMAPF):

    robot_msg = "(Robot"
    for r in robots:
        robot_msg = robot_msg +" " + str(r.name)
    robot_msg = robot_msg +")"
    msg = "(TaskPolicy "+robot_msg+" "+goal[0]+")"
    print("requestToTPL => ", msg)
    res = arbi.request(TPL_address, msg)
    return res

# generate safety&efficiency matrix
def generateSEMatrix(msg):
    tmp = msg.split("(")
    s = tmp[3].split(")")[0].split()[1:]
    e = tmp[4].split(")")[0].split()[1:]
    s_mat = np.ones((len(s), 1))
    e_mat = np.ones((len(e), 1))
    for i in range(len(e)):
        s_mat[i,0] = np.float(s[i])
        e_mat[i,0] = np.float(e[i])

    return s_mat, e_mat


def generateCostMatrix(robotPlans, goals, arbi, arbiMAPF):
    costMat = np.ones((len(robotPlans), len(goals)))
    # iterate to fill matrix
    # for each robot
    serializedCosts = []
    for r in robotPlans:
        for g in goals:
            # name1,start1,goal1;name2,start2,goal2, ...
            # msg = (r.name + "," + r.start + "," + g)
            # send through arbi
            # reqStartTime = time.time()
            # pathMsg = arbi.query(arbiMAPF,msg)
            pathMsg_gl = planMultiAgentReqest([rc.robotPlan(r.name, r.start, g)], arbi, arbiMAPF)
            pathMsg = c.arbi2msg_res(pathMsg_gl)
            # reqEndTime = time.time()
            # print("Request took " + str(reqEndTime - reqStartTime) + " seconds")
            # expect to have result for only one robot
            planResultDict = {}
            if (len(pathMsg) > 0):
                planResultDict = c.responsToDict(pathMsg)
            # fill cost matrix
            # if robot name is matched
            if (r.name in planResultDict):
                # if plan was a success
                if (planResultDict[r.name][0] != 'failed'):
                    serializedCosts.append(len(planResultDict[r.name]))
                else:
                    # path plan failed. apply super large cost
                    serializedCosts.append(c.superLargeCost)
            else:
                # path plan failed. apply super large cost
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

    # fill matrix out of serialized costs
    # by row vectors
    for ind in range(0, len(serializedCosts)):
        row = ind // len(goals)
        col = ind % len(goals)
        costMat[row, col] = serializedCosts[ind]

    return costMat