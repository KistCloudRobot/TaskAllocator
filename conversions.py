import numpy as np

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.agent import arbi_agent_executor
from arbi_agent.model import generalized_list_factory as GLFactory

import time
import deps.printInColor as pic

superLargeCost = 99999999
robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'

def glEx2str(glExpression):
    gl_str = str(glExpression)
    if(gl_str[0]=='\"'):
        return gl_str[1:-1]
    else:
        return gl_str

def str2Glstr(str):
    return ("\"" + str + "\"")


def msg2arbi_req(msg, header="MultiRobotPath", pathHeader = "RobotPath"):
    out_msg = ''
    if(msg == 'failed'):
        out_msg = "(" + header + " " + msg + ")"
    else:
        # name1,start1,goal1;name2,start2,goal2, ...
        # (MultiRobotPath (RobotPath $robot_id $cur_vertex $goal_id), …)
        
        out_msg = "(" + header + " "
        planList = []
        msgList = msg.split(robot_robot_delim)
        for r in msgList:
            # name1,start1,goal1
            #(RobotPath $robot_id $cur_vertex $goal_id) -> append to planList
            elems = r.split(robot_path_delim)
            planList.append('(' + pathHeader + " " + "\"" + elems[0] + "\" " + elems[1] + " " + elems[2] + ')')
        
        out_msg += ' '.join(planList)
        out_msg += ')'

    return out_msg

def arbi2msg_res(arbi_msg,header="MultiRobotPath", pathHeader = "RobotPath", singlePathHeader = "path"):    
    # (MultiRobotPath (RobotPath "agent1" (path 219 220 221 222 223 224 225 15)))
    # name1,start1,goal1;name2,start2,goal2, ...
    gl = GLFactory.new_gl_from_gl_string(arbi_msg)
    robotSet = []
    if(gl.get_name() == header):
        for r in range(gl.get_expression_size()):
            #(RobotPath "agent1" (path 219 220 221 222 223 224 225 15))
            rp = str(gl.get_expression(r))
            #back to gl
            gl_sub = GLFactory.new_gl_from_gl_string(rp)
            robot_name = str(gl_sub.get_expression(0))[1:-1]
            path_list = str(gl_sub.get_expression(1))[1:-1].split(' ')
            #remove gl name
            path_list.pop(0)

            robotSet.append(robot_name+robot_path_delim+path_path_delim.join(path_list))

    return robot_robot_delim.join(robotSet)


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


def makeSqMat(mat):
    n, m = mat.shape
    if m > n:
        dummy_mat = superLargeCost*np.full(((m-n), m), 1)
        mat = np.vstack((mat, dummy_mat))
    elif m < n:
        dummy_mat = superLargeCost*np.full((n, (n-m)), 1)
        mat = np.hstack((mat, dummy_mat))

    return mat
