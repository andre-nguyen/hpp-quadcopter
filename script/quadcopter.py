from hpp.tp_rrt import Robot

robot = Robot("buggy")
robot.setJointBounds ("base_joint_xy", [-5, 16, -4.5, 4.5])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import Viewer
gui = Viewer (ps)

gui.loadObstacleModel ('tp-rrt', "scene", "scene")

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = q_init[0:2]=[-3.7, -4]; gui(q_init)
gui (q_init)

q_goal [0:2] = [15,2]
gui (q_goal)

#~ ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.selectPathPlanner ("PlannerTP")
ps.addPathOptimizer ("RandomShortcut")

t = ps.solve ()
print ("solving time", t)


from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, gui)

