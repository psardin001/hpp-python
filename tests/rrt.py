from motion_planner import MotionPlanner
from pyhpp.gepetto import Viewer
import numpy as np
from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf

urdfFilename = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
srdfFilename = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"

robot = Device.create("ur5")

viewer = Viewer("construction_set", robot)

viewer.addURDFToScene(0, "r0", "anchor", urdfFilename, srdfFilename, SE3.Identity())


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
