import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np

dimension=100
count=100
height=1
rospy.init_node('insert_object',log_level=rospy.INFO)
f_red = open('/home/marko/catkin_ws/src/larics_gazebo_worlds/models/red_box/model.sdf','r')
sdff_red = f_red.read()

f_green = open('/home/marko/catkin_ws/src/larics_gazebo_worlds/models/green_box/model.sdf','r')
sdff_green = f_green.read()

f_blue = open('/home/marko/catkin_ws/src/larics_gazebo_worlds/models/blue_box/model.sdf','r')
sdff_blue = f_blue.read()
rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
for i in range(count):
    color=np.random.randint(0, 3)
    x=np.random.randint(0, dimension)
    y=np.random.randint(0, dimension)
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = height
    if color==1:
        spawn_model_prox("box"+str(i), sdff_red, "boxes_ns", initial_pose, "world")
    elif color==2:
        spawn_model_prox("box"+str(i), sdff_green, "boxes_ns", initial_pose, "world")
    else:
        spawn_model_prox("box"+str(i), sdff_blue, "boxes_ns", initial_pose, "world")