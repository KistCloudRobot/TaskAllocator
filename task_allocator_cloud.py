import sys
import time
import pathlib

sys.path.append("/home/kist/pythonProject/Python-mcArbiFramework")

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.agent import arbi_agent_executor
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.model import generalized_list_factory
from arbi_agent.configuration import BrokerType

import deps.printInColor as pic

import conversions as c
import allocation as alloc

import robotPlan_class as rc

from log.setup import logger

# robot_path_delim = ':'
# robot_robot_delim = ';'
# path_path_delim = '-'
arbiNavManager = "agent://www.arbi.com/NavagiationController"
arbiMAPF = "agent://www.arbi.com/MultiAgentPathFinder"
data_source_name = "ds://www.arbi.com/TaskAllocator"

alloc_gl_name = 'TaskAllocation'
out_gl_name = 'AgentRecommended'

# robotMap = {"lift": ["AMR_LIFT1", "AMR_LIFT2"]}
robotMap = {"lift": ["AMR_LIFT1", "AMR_LIFT2", "AMR_LIFT3", "AMR_LIFT4"]}

agentName = "agent://www.arbi.com/TaskAllocator"
# brokerURL = "tcp://127.0.0.1:61316"
brokerURL = "tcp://192.168.100.10:61316"


class TaskAllocatorDataSource(DataSource):
    def __init__(self, aAgent):
        self.task_allocator = aAgent


class aAgent(ArbiAgent):
    def __init__(self, agent_name, broker_url="tcp://127.0.0.1:61316"):
        super().__init__()
        self.broker_url = broker_url
        self.agent_name = agent_name
        self.data_source = TaskAllocatorDataSource(self)
        self.data_source.connect(broker_url, data_source_name, BrokerType.ZERO_MQ)
        # self.agent_url = agent_url

    def on_data(self, sender: str, data: str):
        print(self.agent_url + "\t-> receive data : " + data)

    def on_request(self, sender: str, request: str) -> str:
        print(self.agent_url + "\t-> receive request : " + request)
        # return "(request ok)"
        return handleRequest(request)

    """
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
    """

    def on_query(self, sender: str, query: str) -> str:
        print(self.agent_url + "\t-> receive query : " + query)
        # print(query)
        return "(query ok)"

    def execute(self, broker_type=2):
        arbi_agent_executor.execute(self.broker_url, self.agent_name, self, broker_type=BrokerType.ZERO_MQ,
                                    daemon=False)
        print(self.agent_name + " ready")


def handleRequest(msg_gl):
    # (TaskAllocation $role (goal (metadata $goalID) $goalName (argument $arg1 $arg2 ...)))
    # keep all the other info in str, take goals out (picking)
    # figure out all applicable robot id from somewhere. Currently predefined
    # request robot avilability from map manager and remove unavailable robot, then do planning
    corr_robots = []
    goalID = ""  # keep and return as it is
    goalName = ""  # keep and return as it is
    goalArgs = []  # this stores target
    gl = generalized_list_factory.new_gl_from_gl_string(msg_gl)
    gl_name = gl.get_name()
    if (gl_name == alloc_gl_name):
        # corr_robots, goalID, goalName, goalArgs, robotPlanSet, goals = parseTMreq(gl)
        # It will get current positions of all robots as well
        corr_robots, goalID, robotPlanSet, goals, is_station_dict = parseTMreq(gl)
        # assumeing for test
        # robotPlan(robot_id,current_vertex(in str))
        # for test
        # robots = (robotPlan("agent1","219"),robotPlan("agent2","222"));
        if (len(robotPlanSet) == 0):
            # no robot is available. return no allocation here
            # return ("("+ out_gl_name + "\"failed\" " + str2Glstr(goalID) + ")")
            return ("(" + out_gl_name + "\"failed\")")
        robots = robotPlanSet

        # fill cost matrix
        # n by m matrix. n = n of robots (rows), m = number of goals(cols)
        # cost_mat = np.random.rand(nRobots, nRobots*numWays)*10
        allocRobotPlans = alloc.allocationCore(robots, goals, arbiAgent, arbiMAPF)
        # final Multi Agent plan Request

        return generate_TM_response(allocRobotPlans, goalID, is_station_dict)
        # return finalResult_gl
        # toss it to other ARBI Agent
        # arbiAgent.send(arbiNavManager, finalResult)
    else:
        pic.printC("Unknown Request GL Name " + gl_name, 'fail')
        return


def parseTMreq(gl):
    print('parseTmReq')
    # (TaskAllocation $goal1 $goal2 ….)
    # goal = (PalletTransported $goalID $station) | (MovingRackTransported $goalID $station)
    num_of_goals = gl.get_expression_size()
    print('num_of_goals' + str(num_of_goals))
    # get all robot names
    # robot_names_all = robotMap["lift"] + robotMap["tow"]
    robot_names_all = robotMap["lift"]

    # generate goalID dict / goals list
    goalID = {}
    goals = []

    is_station_dict = {}

    # iterate goal gls
    for g_ind in range(num_of_goals):
        print(g_ind)
        goal_gl = gl.get_expression(g_ind)
        role = goal_gl.as_generalized_list().get_name()  # not currently in use
        # add to goalID dict
        goalID_in = c.glEx2str(goal_gl.as_generalized_list().get_expression(0))
        station_in = c.glEx2str(goal_gl.as_generalized_list().get_expression(1))
        goalID[station_in] = goalID_in
        # add to goals list
        # remove "station" string
        arg_sp = station_in.split('station')
        if (len(arg_sp) == 2):
            goals.append(arg_sp[1])
            is_station_dict[arg_sp[1]] = True
        else:
            pic.printC("Goal name does not contain \"station\"", 'warning')
            goals.append(arg_sp[0])
            is_station_dict[arg_sp[0]] = False

    # use all robots for all plannings for now
    corr_robots = robot_names_all

    robotPlanSet = []
    for r in corr_robots:

        # get robot status from ltm
        print("start : " + str(r))
        query_robot_at = "(context (robotAt \"" + str(r) + "\" $v1 $v2))"
        print(1)
        query_result_robot_at = arbiAgent.data_source.retrieve_fact(query_robot_at)
        print(2)
        gl_query_result_robot_at = generalized_list_factory.new_gl_from_gl_string(query_result_robot_at)
        print(3)
        v = gl_query_result_robot_at.get_expression(0).as_generalized_list().get_expression(1).as_value().int_value()
        print(4)
        query_robot_loading = "(context (robotLoading \"" + str(r) + "\" $v1))"
        print(5)
        print(query_robot_loading)
        query_result_robot_loading = arbiAgent.data_source.retrieve_fact(query_robot_loading)
        print(query_result_robot_loading)
        if (query_result_robot_loading == "(error)"):
            pass
        else:
            gl_query_result_robot_loading = generalized_list_factory.new_gl_from_gl_string(query_result_robot_loading)
            loading = gl_query_result_robot_loading.get_expression(0).as_generalized_list().get_expression(
                1).as_value().string_value()

        query_robot_task = "(goalAssigned $v1 \"agent://www.mcarbi.com/" + str(r) + "\")"
        query_result_robot_task = arbiAgent.data_source.retrieve_fact(query_robot_task)
        print(query_result_robot_task)
        if query_result_robot_task == "(error)":
            print(str(r) + ' is not working')
            goal_id = 0
        else:
            gl_query_result_robot_task = generalized_list_factory.new_gl_from_gl_string(query_result_robot_task)
            goal_id = gl_query_result_robot_task.get_expression(0).as_value().string_value()

        if goal_id == 0:
            print(6)
            robotPlanSet.append(rc.robotPlan(r, v))
            print(robotPlanSet)
        print("finished")

    # return corr_robots, goalID, goalName, goalArgs, robotPlanSet, goals[0] #assume this GL constains only one task allocation request
    return corr_robots, goalID, robotPlanSet, goals, is_station_dict


def generate_TM_response(allocRobotPlans, goalID, is_station_dict):
    # final Multi Agent plan Request
    print('generate TM response')
    finalResult_gl = alloc.planMultiAgentReqest(allocRobotPlans, arbiAgent, arbiMAPF)
    finalResult = c.arbi2msg_res(finalResult_gl)
    if (finalResult == ''):  # failed
        return c.msg2arbi_req('failed')
    # pic.printC("Allocation Result: " + finalResult, 'cyan')
    logger.info("Allocation Result: " + finalResult, 'cyan')
    finalResult_list_by_robot = finalResult.split(c.robot_robot_delim)
    out_id_goal_pair_list = []
    for robot in finalResult_list_by_robot:
        robot_sp = robot.split(c.robot_path_delim)
        robot_id_out = robot_sp[0]
        goal_vertex = robot_sp[1].split(c.path_path_delim)[-1]
        out_id_goal_pair_list.append((robot_id_out, goal_vertex))
    # generate returning gl string
    # currently assuming there is only one task allocation each time
    out_str = "(" + out_gl_name
    for pair in out_id_goal_pair_list:
        # out_str+=(" " + str2Glstr(pair[0]) + " " + goalID)
        station_str = ""
        if (is_station_dict[pair[1]] == True):
            station_str = "http://www.arbi.com/ontologies/arbi.owl#station"
        out_str += (" (Allocation " + "\"" + str(goalID[station_str + str(pair[1])]) + "\" \"" + str(pair[0]) + "\")")
    out_str += ")"
    return out_str


if __name__ == "__main__":
    print("custom")
    # Create and start an ARBI Agent
    if len(sys.argv) > 1:
        ip = sys.argv[1]
    else:
        ip = brokerURL
    global arbiAgent
    arbiAgent = aAgent(agent_name=agentName, broker_url=ip)
    arbiAgent.execute()
    '''
    response = arbiAgent.request(arbiTaskAllocator,"(TaskAllocation (PalletTransported \"Local1\" \"http://www.arbi.com/ontologies/arbi.owl#station1\"))")
    print("response is " + str(response))
    response = arbiAgent.request(arbiTaskAllocator,
                                 "((TaskAllocation (PalletTransported \"Local2\" \"http://www.arbi.com/ontologies/arbi.owl#station20\"))")
    print("response is " + str(response))
    '''
#
#    while (1):
#        time.sleep(0.01)
