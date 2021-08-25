import matching
import numpy as np

numWays = 1
n=3
cost_mat = np.random.rand(n, n*numWays)*10
assignment, cost = matching.matching(cost_mat, numWays)

print("***RESULT (numWays = %d)***\n" %numWays)
print("The cost matrix")
print(cost_mat)
print("\nThe optimal allocation")
print(assignment.astype("int"))
print("\nThe cost sum: %f" %cost)

numWays = 2
n = 3
cost_mat = np.random.rand(n, n*numWays)*10
assignment, cost = matching.matching(cost_mat, numWays)

print("\n***RESULT (numWays = %d)***\n" %numWays)
print("The cost matrix")
print(cost_mat)
print("\nThe optimal allocation")
print(assignment.astype("int"))
print("\nThe cost sum: %f" %cost)

bigC = 1000
numWays = 1
n = 2
m = 1
cost_mat = np.random.rand(n, m*numWays)*10

if m > n:
    dummy_mat = bigC*np.full(((m-n), numWays*m), 1)
    cost_mat = np.vstack((cost_mat, dummy_mat))
elif m < n:
    dummy_mat = bigC*np.full((n, (n-m)), 1)
    cost_mat = np.hstack((cost_mat, dummy_mat))
    
assignment, cost = matching.matching(cost_mat, numWays)
if m > n:
    assignment[n:, :] = False
    cost = cost-bigC*(m-n)
elif m < n:
    assignment[:, m:] = False
    cost = cost-bigC*(n-m)
    
print("\n***RESULT (numWays = %d)***\n" %numWays)
print("The cost matrix")
print(cost_mat)
print("\nThe optimal allocation")
print(assignment.astype("int"))
print("\nThe cost sum: %f" %cost)
