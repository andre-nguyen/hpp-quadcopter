from hpp.quadcopter import Robot

robot = Robot("quad")
robot.client.robot.setDimensionExtraConfigSpace(6)
robot.setJointBounds ("base_joint_xyz", [-5, 16, -4.5, 4.5, 0, 0.6])
robot.client.robot.setExtraConfigSpaceBounds([-1,1,-1,1,-1,1,-1,1,-1,1,-1,1])

from hpp.corbaserver import ProblemSolver
ps = ProblemSolver (robot)

from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

vf.loadObstacleModel ('hpp-quadcopter', "scene", "scene")

q_init = robot.getCurrentConfig ()
q_init[2] = 0.5
q_goal = q_init [::]
q_init [0:2] = [-3.7, -4];
vf (q_init)

q_goal [0:2] = [15,2]
vf (q_goal)

#~ ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.selectSteeringMethod("Quadcopter")

# ps.addPathOptimizer ("RandomShortcut")

# t = ps.solve ()
# print ("solving time", t)


# gui = vf.createViewer()
# from hpp.gepetto import PathPlayer
# pp = PathPlayer (robot.client, gui)

