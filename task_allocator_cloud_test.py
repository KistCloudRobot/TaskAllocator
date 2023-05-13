import numpy as np
import yaml
import deps.matching as matching
import cloud_MAPF as mapf

#############################
## from conversions.py
#############################
superLargeCost = 99999999

def makeSqMat(mat):
    n, m = mat.shape
    if m > n:
        dummy_mat = superLargeCost * np.full(((m - n), m), 1)
        mat = np.vstack((mat, dummy_mat))
    elif m < n:
        dummy_mat = superLargeCost * np.full((n, (n - m)), 1)
        mat = np.hstack((mat, dummy_mat))

    return mat

#############################
## from allocation.py
#############################
def generateCostMatrix(robotPlans, goals):
    print_result = False

    costMat = np.ones((len(robotPlans), len(goals)))
    # iterate to fill matrix
    # for each robot
    serializedCosts = []
    for r in robotPlans:
        for g in goals:
            # plan request to mapf
            planResultDict = mapf.planMultiAgentReqest([[r['name'], r['start'], g]], print_result)

            # fill cost matrix
            # if robot name is matched
            if (r['name'] in planResultDict):
                # if plan was a success
                if (planResultDict[r['name']][0] != 'failed'):
                    serializedCosts.append(len(planResultDict[r['name']]))
                else:
                    # path plan failed. apply super large cost
                    serializedCosts.append(superLargeCost)
            else:
                # path plan failed. apply super large cost
                serializedCosts.append(superLargeCost)

    # fill matrix out of serialized costs
    # by row vectors
    for ind in range(0, len(serializedCosts)):
        row = ind // len(goals)
        col = ind % len(goals)
        costMat[row, col] = serializedCosts[ind]

    return costMat


def allocationCore(robots, goals):
    # create and fill cost matrix
    cost_mat = generateCostMatrix(robots, goals)

    # fill missing rows/cols to make it square
    cost_mat = makeSqMat(cost_mat)
    
    # assignment, cost = matching.matching(cost_mat, numWays)
    assignment, cost = matching.matching(cost_mat)
    allocMat = assignment.astype("int")

    # remove allocation with super large cost
    c_rows, c_cols = cost_mat.shape
    for i in range(c_rows):
        for j in range(c_cols):
            if (cost_mat[i, j] > (superLargeCost - 1)):
                allocMat[i, j] = int(0)

    # print("***RESULT (numWays = %d)***\n" %numWays)
    print("***RESULT***\n")
    print("The cost matrix")
    print(cost_mat)
    print("\nThe optimal allocation")
    print(allocMat)
    print("\nThe cost sum: %f" % cost)

    # consider not allocated agents
    # iterate matrix to extract allocation result
    # result = name1:node1,node2, ... ,noden;name2:node1,node2, ... ,noden
    allocRobotPlans = []
    for row in range(len(robots)):
        for col in range(len(goals)):
                if (allocMat[row][col] == 1):
                    robots[row]['goal'] = str(goals[col])
                    allocRobotPlans.append(robots[row])

    return allocRobotPlans


#############################
## main
#############################
def test():
    robots = []
    robots.append({'name': 'AMR_LIFT1', 'start':'143'})
    robots.append({'name': 'AMR_LIFT2', 'start':'158'})
    goals = ['12', '46']
    allocRobotPlans = allocationCore(robots, goals)
    print('result: ', allocRobotPlans)


def main():
    scenario_file_path = 'scen/scen1.yaml'

    param = None
    with open(scenario_file_path, 'r') as arg:
        try:
            param = yaml.load(arg, Loader=yaml.BaseLoader)
        except yaml.YAMLError as exc:
            print(exc)

    robots = param['robots']
    goals = param['goals']
    allocRobotPlans = allocationCore(robots, goals)
    print('result: ', allocRobotPlans)
    

if __name__ == "__main__":
    # test()
    main()