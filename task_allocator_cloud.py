import matching
import numpy as np

from python_arbi_framework.arbi_agent.agent.arbi_agent import ArbiAgent
from python_arbi_framework.arbi_agent.configuration import BrokerType
from python_arbi_framework.arbi_agent.agent import arbi_agent_excutor

import time

import printInColor as pic

superLargeCost = 99999999
robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'
arbiNavManager = "agent://www.arbi.com/navManager"
arbiMAPF = "agent://www.arbi.com/MAPFagent"


class aAgent(ArbiAgent):
    def __init__(self, agent_name, broker_url = "tcp://127.0.0.1:61616"):
        super().__init__()
        self.broker_url = broker_url
        self.agent_name = agent_name
        #self.agent_url = agent_url

    def on_data(self, sender: str, data: str):
        print(self.agent_url + "\t-> receive data : " + data)
    
    def on_request(self, sender: str, request: str) -> str:
        print(self.agent_url + "\t-> receive request : " + request)
        return "(request ok)"
    
    """
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
    """
    def on_query(self, sender: str, query: str) -> str:
        print(self.agent_url + "\t-> receive query : " + query)
        #print(query)
        return "(query ok)"

    def execute(self, broker_type=2):
        arbi_agent_excutor.excute(self.broker_url, self.agent_name, self, broker_type)
        print(self.agent_name + " ready")



class robotPlan:
    def __init__(self,name,start,goal=""):
        self.name = name
        self.start = start
        self.goal = goal

"""
def robotPlanList_to_Msg(rpList):
    #result = name1:node1,node2, ... ,node[n];name2:node1,node2, ... ,node[n]
    msgByRobot = []
    for rp in rpList:
        singleMsg = rp.name + ":" + 
"""

def responsToDict(res):
    out_dict = {}
    #robots are separated by robot_robot_delim
    resByRobots = res.split(robot_robot_delim)
    for pathMsg in resByRobots:
        #name and path nodes are separated by robot_path_delim
        pathMsg_sp = pathMsg.split(robot_path_delim)
        #value could be 'failed'. It will be turned to ['failed']
        path_list = pathMsg_sp[1].split(path_path_delim)
        out_dict[pathMsg_sp[0]] = path_list

    return out_dict
       
       
def planMultiAgentReqest(robotPlans, arbi):
    msgByRobot = []
    for r in robotPlans:
        if(len(r.goal) > 0):
            singleMsg = (r.name + robot_path_delim + r.start + robot_path_delim + r.goal)
            msgByRobot.append(singleMsg)
    reqMsg = robot_robot_delim.join(msgByRobot)
    reqStartTime = time.time()
    res = arbi.request(arbiMAPF,reqMsg)
    reqEndTime = time.time()
    pic.printC("Request took " + str(reqEndTime - reqStartTime) + " seconds", 'warning')
    return res


def generateCostMatrix(robotPlans,goals,arbi):
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
            pathMsg = planMultiAgentReqest([robotPlan(r.name,r.start,g)],arbi)
            #reqEndTime = time.time()
            #print("Request took " + str(reqEndTime - reqStartTime) + " seconds")
            #expect to have result for only one robot
            planResultDict = responsToDict(pathMsg)
            #fill cost matrix
            #if robot name is matched
            if(r.name in planResultDict):
                #if plan was a success
                if(planResultDict[r.name][0] != 'failed'):
                    serializedCosts.append(len(planResultDict[r.name]))
                else:
                    #path plan failed. apply super large cost
                    serializedCosts.append(superLargeCost)
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


#Create and start an ARBI Agent
arbiAgent = aAgent("agent://www.arbi.com/TA")
arbiAgent.execute()

while(1):
    print("Press a Key to Begin a Test Run")
    input()
    #one path to one target
    #numWays = 1
    #number of robots
    #nRobots=2

    #get number of goals from somewhere
    #nGoals = 2
    goals = ("15","1")
    #assumeing for test
    robots = (robotPlan("agent1","219"),robotPlan("agent2","222"));
    #fill cost matrix
    #n by m matrix. n = n of robots (rows), m = number of goals(cols)
    #cost_mat = np.random.rand(nRobots, nRobots*numWays)*10   

    cost_mat = generateCostMatrix(robots,goals,arbiAgent)

    #assignment, cost = matching.matching(cost_mat, numWays)
    assignment, cost = matching.matching(cost_mat)

    allocMat = assignment.astype("int")

    #print("***RESULT (numWays = %d)***\n" %numWays)
    print("***RESULT***\n")
    print("The cost matrix")
    print(cost_mat)
    print("\nThe optimal allocation")
    print(allocMat)
    print("\nThe cost sum: %f" %cost)
    
    #iterate matrix to extract allocation result
    #result = name1:node1,node2, ... ,noden;name2:node1,node2, ... ,noden
    allocRobotPlans = []
    for row in range(len(robots)):
        for col in range(len(goals)):
            if(allocMat[row][col] == 1):
                robots[row].goal = goals[col]
                allocRobotPlans.append(robots[row])

    #final Multi Agent plan Request
    finalResult = planMultiAgentReqest(allocRobotPlans,arbiAgent)
    pic.printC("Allocation Result: " + finalResult,'cyan')
    #toss it to other ARBI Agent
    arbiAgent.send(arbiNavManager, finalResult)

