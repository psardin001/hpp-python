from motion_planner import MotionPlanner
from pyhpp.gepetto import Viewer
import numpy as np
from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import Problem

urdfFilename = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
srdfFilename = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"

robot = Device.create("ur5")

viewer = Viewer("construction_set", robot)

viewer.addURDFToScene(0, "r0", "anchor", urdfFilename, srdfFilename, SE3.Identity())

viewer.addURDFObstacleToScene("package://hpp_practicals/urdf/ur_benchmark/obstacles.urdf", "obstacles")
viewer.addURDFObstacleToScene("package://hpp_practicals/urdf/ur_benchmark/table.urdf", "table")
viewer.addURDFObstacleToScene("package://hpp_practicals/urdf/ur_benchmark/wall.urdf", "wall")

q2 = np.array([0.2, -1.57, -1.8, 0, 0.8, 0])

viewer.applyConfiguration(q2)


# vf.loadObstacleModel(
#     "package://hpp_practicals/urdf/ur_benchmark/obstacles.urdf", "obstacles"
# )
# vf.loadObstacleModel("package://hpp_practicals/urdf/ur_benchmark/table.urdf", "table")
# vf.loadObstacleModel("package://hpp_practicals/urdf/ur_benchmark/wall.urdf", "wall")

# q1 = [0, -1.57, 1.57, 0, 0, 0]
# q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]
# q3 = [1.57, -1.57, -1.8, 0, 0.8, 0]

# ps.setInitialConfig(q2)
# ps.addGoalConfig(q3)


# m = MotionPlanner(robot, ps)
# pathId = m.solveBiRRT(maxIter=1000)

#############RRT############
#def solveBiRRT(self, maxIter=float("inf")):
#    #cpp#
#    initProblem();
#    pathPlanner_->startSolve();
#    pathPlanner_->tryConnectInitAndGoals();
#    roadmap_->pathExists();
#    #cpp#
#    finished = False
#    iter = 0
#    ps = self.ps
#    while True:
#        # RRT begin
#        # Extend
#        newNodes = list()
#        newEdges = list()
#        q_rand = robot.shootRandomConfig()
#        for i in range(ps.numberConnectedComponents()):
#            q_near, d = ps.getNearestConfig(q_rand,i)
#            res, pid, msg = ps.directPath(q_near, q_rand, True)
#            if res:
#                q_new = q_rand
#            else:
#                q_new = ps.configAtParam(pid, ps.pathLength(pid))
#            newNodes.append(q_new)
#            newEdges.append((q_near, q_new, pid))
#        for q in newNodes:
#            ps.addConfigToRoadmap(q)
#        for q1, q2, pid in newEdges:
#            ps.addEdgeToRoadmap(q1, q2, pid, True)
#        # connect
#        for q_new in newNodes:
#            for i in range(ps.numberConnectedComponents()):
#                q_near, d = ps.getNearestConfig(q_new, i)
#                # if q_near == q_new, q_new is in this connected component
#                if q_near != q_new:
#                    res, pid, msg = ps.directPath(q_new, q_near, True)
#                    if res:
#                        ps.addEdgeToRoadmap(q_new, q_near, pid, True)
#                        print('finished')
#                        break
#        # RRT end
#        # Check if the problem is solved.
#        nbCC = self.ps.numberConnectedComponents()
#        if nbCC == 1:
#            # Problem solved
#            finished = True
#            break
#        iter = iter + 1
#        if iter > maxIter:
#            break
#    if finished:
#        self.ps.finishSolveStepByStep()
#        return self.ps.numberPaths() - 1
#
#