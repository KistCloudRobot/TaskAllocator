class robotPlan:
    def __init__(self,name,start,goal=""):
        self.name = name
        self.start = start
        if goal is not None:
            self.goal = str(goal)
