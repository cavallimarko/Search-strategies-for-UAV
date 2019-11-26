#!/usr/bin/env python

import copy, time

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Pose
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button, RadioButtons,Rectangle
import math
from datetime import datetime


#x = np.random.uniform(0, 1, 10)
#y = np.random.uniform(0, 1, 10)
#right 1,0
#down 0,-1
#left -1,0
#up 0,1
#ccw rotation za 90
#x=a+b-y
#y=x+b-a

class RequestTrajectory():

    def __init__(self,points,home=False):
        self.UAVCurrentPosition = Pose()
        # UAV current position topic
        rospy.Subscriber("uav/odometry", Odometry, self.poseCallback)
        # First set up service
        request_trajectory_service = rospy.ServiceProxy(
            "uav/generate_toppra_trajectory", GenerateTrajectory)

        # This is an example for UAV and output trajectory is converted 
        # accordingly
        trajectory_pub = rospy.Publisher('uav/multi_dof_trajectory', 
            MultiDOFJointTrajectory, queue_size=1)
        time.sleep(1)
        #print(self.UAVCurrentPosition)
        #while(self.UAVCurrentPosition):
        #        rate = rospy.Rate(2) # 10hz
        #        rate.sleep()

        if home is True:
            request = GenerateTrajectoryRequest()

            waypoint = JointTrajectoryPoint()

            # Positions are defined above
            waypoint.positions = [self.UAVCurrentPosition.position.x, self.UAVCurrentPosition.position.y, self.UAVCurrentPosition.position.z, 1]
            #waypoint.positions = [x[i], y[i], z[i], yaw[i]]
            waypoint.velocities = [5, 5, 5, 1]
            waypoint.accelerations = [0.75, 0.75, 0.75, 1]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

            self.end=Pose()
            self.end.position.x=0
            self.end.position.y=0
            self.end.position.z=1
            #print(self.end)
            if(self.euclideanDistance(self.UAVCurrentPosition,self.end)<0.1):
                print("UAV arrived at the starting position")
                return
            # Add waypoints in request
            waypoint1 = JointTrajectoryPoint()

            # Positions are defined above
            waypoint1.positions = [0, 0, 1, 0]
            # Also add constraints for velocity and acceleration. These
            # constraints are added only on the first waypoint since the
            # TOPP-RA reads them only from there.
            #if i==0:
            #    waypoint.velocities = [2, 2, 2, 1]
            #    waypoint.accelerations = [1.25, 1.25, 1.25, 1]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint1))

            # Set up joint names. This step is not necessary
            request.waypoints.joint_names = ["x", "y", "z", "yaw"]
            # Set up sampling frequency of output trajectory.
            request.sampling_frequency = 100.0
            # If you want to plot Maximum Velocity Curve and accelerations you can
            # send True in this field. This is intended to be used only when you
            # have to debug something since it will block the service until plot
            # is closed.
            request.plot = False
            # Request the trajectory
            response = request_trajectory_service(request)

            # Response will have trajectory and bool variable success. If for some
            # reason the trajectory was not able to be planned or the configuration
            # was incomplete or wrong it will return False.

            print "Converting trajectory to multi dof"
            joint_trajectory = response.trajectory
            multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
            trajectory_pub.publish(multi_dof_trajectory)
            print("From: ["+
                    str(round(self.UAVCurrentPosition.position.x,2))+", "+
                    str(round(self.UAVCurrentPosition.position.y,2))+", "+
                    str(round(self.UAVCurrentPosition.position.z,2))+
                        "]to: ["+
                    str(0)+", "+
                    str(0)+", "+
                    str(1)+"]"+
                    "Distance="+str(round(self.euclideanDistance(self.UAVCurrentPosition,self.end),2))+"m"
                    )
            while(self.euclideanDistance(self.UAVCurrentPosition,self.end)>0.1):
                print("Distance to home: "+str(round(self.euclideanDistance(self.UAVCurrentPosition,self.end),2))+"m")
                rate = rospy.Rate(2) # 10hz
                rate.sleep()
            print("UAV arrived at the starting position")
        else:
            x=np.transpose(points[:, 0])
            y=np.transpose(points[:, 1])
            z=np.transpose(points[:, 2])
            yaw=np.transpose(points[:, 3])

            start_time = datetime.now()
            print("UAV search started")
            
            for i in range(0, len(x)):

                

                # Create a service request which will be filled with waypoints
                request = GenerateTrajectoryRequest()

                waypoint = JointTrajectoryPoint()

                # Positions are defined above
                waypoint.positions = [self.UAVCurrentPosition.position.x, self.UAVCurrentPosition.position.y, self.UAVCurrentPosition.position.z, yaw[i]]
                #waypoint.positions = [x[i], y[i], z[i], yaw[i]]
                waypoint.velocities = [5, 5, 5, 1]
                waypoint.accelerations = [1.25, 1.25, 1.25, 1]
                #print(waypoint)
                # Append all waypoints in request
                request.waypoints.points.append(copy.deepcopy(waypoint))

                self.end=Pose()
                self.end.position.x=x[i]
                self.end.position.y=y[i]
                self.end.position.z=z[i]
                if(self.euclideanDistance(self.UAVCurrentPosition,self.end)<0.1):
                    print("Start location is equal to the first waypoint")
                    continue
                # Add waypoints in request
                waypoint1 = JointTrajectoryPoint()

                # Positions are defined above
                waypoint1.positions = [x[i], y[i], z[i], yaw[i]]
                # Also add constraints for velocity and acceleration. These
                # constraints are added only on the first waypoint since the
                # TOPP-RA reads them only from there.
                #if i==0:
                #    waypoint.velocities = [2, 2, 2, 1]
                #    waypoint.accelerations = [1.25, 1.25, 1.25, 1]

                # Append all waypoints in request
                request.waypoints.points.append(copy.deepcopy(waypoint1))

                # Set up joint names. This step is not necessary
                request.waypoints.joint_names = ["x", "y", "z", "yaw"]
                # Set up sampling frequency of output trajectory.
                request.sampling_frequency = 100.0
                # If you want to plot Maximum Velocity Curve and accelerations you can
                # send True in this field. This is intended to be used only when you
                # have to debug something since it will block the service until plot
                # is closed.
                request.plot = False

                # Request the trajectory
                response = request_trajectory_service(request)

                # Response will have trajectory and bool variable success. If for some
                # reason the trajectory was not able to be planned or the configuration
                # was incomplete or wrong it will return False.

                print "Converting trajectory to multi dof"
                joint_trajectory = response.trajectory
                multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
                trajectory_pub.publish(multi_dof_trajectory)
                print("From: ["+
                    str(round(self.UAVCurrentPosition.position.x,2))+", "+
                    str(round(self.UAVCurrentPosition.position.y,2))+", "+
                    str(round(self.UAVCurrentPosition.position.z,2))+
                        "]to: ["+
                    str(round(x[i],2))+", "+
                    str(round(y[i],2))+", "+
                    str(round(z[i],2))
                    +"]"+
                    " Distance= "+str(round(self.euclideanDistance(self.UAVCurrentPosition,self.end),2))+"m"
                    )
                while(self.euclideanDistance(self.UAVCurrentPosition,self.end)>0.04):
                    print(str(i+1)+"/"+str(len(x))+" next in: "+str(round(self.euclideanDistance(self.UAVCurrentPosition,self.end),2))+"m")
                    rate = rospy.Rate(2) # 10hz
                    rate.sleep()
                rate = rospy.Rate(1) # 10hz
                rate.sleep()
                print("UAV arrived at the "+str(i+1)+"/"+str(len(x))+" waypoint")
                current_time = datetime.now()
                print("Elapsed time: {}".format(current_time - start_time))
            
            end_time = datetime.now()

            print("UAV search completed in duration: {}".format(end_time - start_time))

    def initcopy():
        # Create a service request which will be filled with waypoints
        request = GenerateTrajectoryRequest()

        # Add waypoints in request
        waypoint = JointTrajectoryPoint()
        for i in range(0, len(x)):
            # Positions are defined above
            waypoint.positions = [x[i], y[i], z[i], yaw[i]]
            # Also add constraints for velocity and acceleration. These
            # constraints are added only on the first waypoint since the
            # TOPP-RA reads them only from there.
            if i==0:
                waypoint.velocities = [2, 2, 2, 1]
                waypoint.accelerations = [1.25, 1.25, 1.25, 1]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

        # Set up joint names. This step is not necessary
        request.waypoints.joint_names = ["x", "y", "z", "yaw"]
        # Set up sampling frequency of output trajectory.
        request.sampling_frequency = 100.0
        # If you want to plot Maximum Velocity Curve and accelerations you can
        # send True in this field. This is intended to be used only when you
        # have to debug something since it will block the service until plot
        # is closed.
        request.plot = False
        # Request the trajectory
        response = request_trajectory_service(request)

        # Response will have trajectory and bool variable success. If for some
        # reason the trajectory was not able to be planned or the configuration
        # was incomplete or wrong it will return False.

        print "Converting trajectory to multi dof"
        joint_trajectory = response.trajectory
        multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
        trajectory_pub.publish(multi_dof_trajectory)
    def euclideanDistance(self,start,end):

        return math.sqrt(
                        math.pow((start.position.x - 
                                  end.position.x), 2) +
                        math.pow((start.position.y -
                                  end.position.y), 2) +
                        math.pow((start.position.z - 
                                  end.position.z), 2)
                    )

    def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.w = 1.0

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory
    def poseCallback(self, msg):
        # Note: orientation is quaternion
        self.UAVCurrentPosition.position.x = msg.pose.pose.position.x
        self.UAVCurrentPosition.position.y = msg.pose.pose.position.y
        self.UAVCurrentPosition.position.z = msg.pose.pose.position.z

        self.UAVCurrentPosition.orientation.x = msg.pose.pose.orientation.x
        self.UAVCurrentPosition.orientation.y = msg.pose.pose.orientation.y
        self.UAVCurrentPosition.orientation.z = msg.pose.pose.orientation.z
        self.UAVCurrentPosition.orientation.w = msg.pose.pose.orientation.w





xs=[]
ys=[]
UAVCurrentPosition = Pose()

fig = plt.figure()
ax = fig.add_subplot(111)
axis_color = 'lightgoldenrodyellow'
#ax.set_xlim([-100, 100])
#ax.set_ylim([-100,100])

length=40     
spacing=20
dimension=100       
sigma=1 

directions=[[1,0],[0,-1],[-1,0],[0,1]]
startinglabel="uniform"
txt=plt.gcf().text(0.1, 0.9, 'Total distance='+"{0:.2f}".format(0)+"m", fontsize=14)
def updatePlot():
    points=generate()
    line.set_xdata(points[:, 0])
    line.set_ydata(points[:, 1])
    [p.remove() for p in reversed(ax.patches)]
    ax.add_patch(Rectangle((0, 0), int(amp_slider2.val), int(amp_slider2.val), alpha=0.2))
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    fig.canvas.draw_idle()
def generate():
    global startinglabel
    spacing=int(amp_slider1.val)
    dimension=int(amp_slider2.val)

    if spacing>dimension*(3./5):
        amp_slider1.set_val(dimension*(3./5))
    points = generatepoints(int(amp_slider.val), amp_slider1.val,int(amp_slider2.val),amp_slider3.val,startinglabel)
   

    return points
def generatepoints(length,spacing,dimension,sigma,startinglabel="uniform",height=3):
    if (spacing < 10e-7):
        spacing = 10e-3
    start = np.array([[0, 0, height, 0]])
    points = np.array(start)
    distance_travelled=0
    global txt
    for i in range(1,int(length)):
        a=points[i-1][0]
        b=points[i-1][1]
        x=a
        y=b
        #direction_x=int(np.floor(np.random.randint(0,4)))
        j=0
        while True:
            x = a
            y = b
            if startinglabel=="uniform":
                angle=np.random.uniform(0,1)*(2*math.pi)
            elif startinglabel=="normal":
                mu=0
                angle=np.random.normal(mu,sigma)
            elif startinglabel=="normal_with_turn":
                mu=0
                angle=np.random.normal(mu,sigma)+(math.pi/125)*i
            elif startinglabel=="normal_with_turn_and_scale":
                mu = 0
                angle = np.random.normal(mu, sigma) + (math.pi / 125) * i
                spacing*=0.999
            elif startinglabel=="levy":
                spacing_prev=spacing
                angle = np.random.uniform(0, 1) * (2 * math.pi)
                r=np.random.uniform(0,1)
                spacing*=np.random.uniform(0,1)**(-1/sigma)
            direction_x=math.cos(angle)
            direction_y=math.sin(angle)
            x=x+direction_x*spacing
            y=y+direction_y*spacing
            if startinglabel=="levy":
                spacing=spacing_prev
            if x >= 0 and x <= dimension and y >= 0 and y <= dimension:
                break

            if(j>=100):
                x = a
                y = b
                break
            j+=1
        points=np.vstack((points,np.array([[x,y, height, 0]])))
        distance_travelled += np.linalg.norm(np.array([a,b])-np.array([x,y]))
        xs.append(x)

        ys.append(y)

    message='Total distance='+"{0:.2f}".format(distance_travelled)+"m"
    txt.set_text(message)

    return points


# Plot non-ordered points
points=generatepoints(length,spacing,dimension,sigma)
[line] = ax.plot(points[:,0], points[:,1], marker="o", markerfacecolor="r", linestyle='-')

#rospy.init_node('insert_object',log_level=rospy.INFO)

def spawn_boxes(dimension=50,count=100,height=1):
    from gazebo_msgs.srv import SpawnModel
    from geometry_msgs.msg import Pose


    

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

def poseCallback(msg):
    global UAVCurrentPosition
    # Note: orientation is quaternion
    UAVCurrentPosition.position.x = msg.pose.pose.position.x
    UAVCurrentPosition.position.y = msg.pose.pose.position.y
    UAVCurrentPosition.position.z = msg.pose.pose.position.z

    UAVCurrentPosition.orientation.x = msg.pose.pose.orientation.x
    UAVCurrentPosition.orientation.y = msg.pose.pose.orientation.y
    UAVCurrentPosition.orientation.z = msg.pose.pose.orientation.z
    UAVCurrentPosition.orientation.w = msg.pose.pose.orientation.w

    for i in range(checkPosesArrayCount-1):
        checkPosesArray[i].pose.position.x = checkPosesArray[i+1].pose.position.x
        checkPosesArray[i].pose.position.y = checkPosesArray[i+1].pose.position.y
        checkPosesArray[i].pose.position.z = checkPosesArray[i+1].pose.position.z

    checkPosesArray[checkPosesArrayCount-1].pose.position.x = UAVCurrentPosition.position.x
    checkPosesArray[checkPosesArrayCount-1].pose.position.y = UAVCurrentPosition.position.y
    checkPosesArray[checkPosesArrayCount-1].pose.position.z = UAVCurrentPosition.position.z

# UAV current position topic
#rospy.Subscriber("uav/pose", Pose, poseCallback)


# Define an action for modifying the line when any slider's value changes
def sliders_on_changed(val):
    updatePlot()
amp_0 = length
amp_slider_ax  = fig.add_axes([0.25, 0.15, 0.65, 0.03], facecolor=axis_color)
amp_slider = Slider(amp_slider_ax, 'Number of steps', 1, 300, valinit=amp_0,valstep=1,valfmt= "%d")
amp_slider.on_changed(sliders_on_changed)

amp_1 = spacing
amp_slider_ax1  = fig.add_axes([0.25, 0.1, 0.65, 0.03], facecolor=axis_color)
amp_slider1 = Slider(amp_slider_ax1, 'Spacing', 0, 50, valinit=amp_1,valfmt= "%.2f m")
amp_slider1.on_changed(sliders_on_changed)

amp_2 = dimension
amp_slider_ax2  = fig.add_axes([0.25, 0.05, 0.65, 0.03], facecolor=axis_color)
amp_slider2 = Slider(amp_slider_ax2, 'Search space dimension', 0, 800, valinit=amp_2,valfmt= "%.2f m")
amp_slider2.on_changed(sliders_on_changed)

amp_3 = sigma
amp_slider_ax3  = fig.add_axes([0.25, 0.0, 0.65, 0.03], facecolor=axis_color)
amp_slider3 = Slider(amp_slider_ax3, 'Sigma/alpha', 0, 5, valinit=amp_3,valfmt= "%.3f")
amp_slider3.on_changed(sliders_on_changed)
# Add a button for resetting the parameters
reset_button_ax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
reset_button = Button(reset_button_ax, 'Reset', color=axis_color, hovercolor='0.975')
def reset_button_on_clicked(mouse_event):
    amp_slider.reset()
    amp_slider1.reset()
    amp_slider2.reset()
reset_button.on_clicked(reset_button_on_clicked)
regenerate_button_ax = fig.add_axes([0.8, 0.95, 0.1, 0.04])
regenerate_button = Button(regenerate_button_ax, 'Regenerate', color=axis_color, hovercolor='0.975')
def regenerate_button_on_clicked(mouse_event):
    updatePlot()
regenerate_button.on_clicked(regenerate_button_on_clicked)

publish_trajectory_button_ax = fig.add_axes([0.8, 0.0, 0.1, 0.04])
publish_trajectory_button = Button(publish_trajectory_button_ax, 'SendTrajectory', color=axis_color, hovercolor='0.975')
def publish_trajectory_button_on_clicked(mouse_event):
    #points=generate()
    rospy.init_node("topp_trajectory_call_example")
    RequestTrajectory(points)
publish_trajectory_button.on_clicked(publish_trajectory_button_on_clicked)

return_home_button_ax = fig.add_axes([0.02, 0.0, 0.1, 0.04])
return_home_button = Button(return_home_button_ax, 'ReturnHome', color=axis_color, hovercolor='0.975')
def return_home_button_on_clicked(mouse_event):
    points=np.array([0,0,1,0])
    rospy.init_node("topp_trajectory_call_example")
    RequestTrajectory(points,home=True)
return_home_button.on_clicked(return_home_button_on_clicked)

spawn_boxes_button_ax = fig.add_axes([0.02, 0.05, 0.1, 0.04])
spawn_boxes_button = Button(spawn_boxes_button_ax, 'SpawnBoxes', color=axis_color, hovercolor='0.975')
def spawn_boxes_button_on_clicked(mouse_event):
    spawn_boxes()
spawn_boxes_button.on_clicked(spawn_boxes_button_on_clicked)


color_radios_ax = fig.add_axes([0.025, 0.5, 0.15, 0.15], facecolor=axis_color)
color_radios = RadioButtons(color_radios_ax, ('uniform', 'normal','normal_with_turn',"normal_with_turn_and_scale","levy"), active=0)
def color_radios_on_clicked(label):
    global startinglabel
    startinglabel=label
    updatePlot()
color_radios.on_clicked(color_radios_on_clicked)
ax.add_patch(Rectangle((0, 0), int(amp_slider2.val), int(amp_slider2.val),alpha=0.2))
plt.show()




