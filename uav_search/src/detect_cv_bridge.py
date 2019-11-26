#!/usr/bin/env python2.7
  # Import ROS libraries and messages
from __future__ import division
import rospy
from sensor_msgs.msg import Image
import math
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Transform, Twist, Pose
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from matplotlib import pyplot as plt
import imutils
from gazebo_msgs.srv import DeleteModel # For deleting models from the environment
import matplotlib.animation as animation

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)
UAVCurrentPosition = Pose()
positions=np.empty((0,2), int)
detection_positions = np.empty((0,3), int)
true_positions = np.empty((0,3), int)
i=0
# UAV current position topic
def poseCallback(msg):
        # Note: orientation is quaternion
        global UAVCurrentPosition
        UAVCurrentPosition.position.x = msg.pose.pose.position.x
        UAVCurrentPosition.position.y = msg.pose.pose.position.y
        UAVCurrentPosition.position.z = msg.pose.pose.position.z

        UAVCurrentPosition.orientation.x = msg.pose.pose.orientation.x
        UAVCurrentPosition.orientation.y = msg.pose.pose.orientation.y
        UAVCurrentPosition.orientation.z = msg.pose.pose.orientation.z
        UAVCurrentPosition.orientation.w = msg.pose.pose.orientation.w
        global i
        global positions

        i+=1
        if i%100==0:
          positions = np.append(positions, np.array([[UAVCurrentPosition.position.x,UAVCurrentPosition.position.y]]), axis=0)

rospy.Subscriber("uav/odometry", Odometry, poseCallback)
# Initialize the CvBridge class
bridge = CvBridge()
def cvSubplot(imgs,     # 2d np array of imgs (each img an np arrays of depth 1 or 3).
              pad=10,   # number of pixels to use for padding between images. must be even
              titles=None,  # (optional) np array of subplot titles
              win_name='CV Subplot' # name of cv2 window
              ):
    '''
    Makes cv2 based subplots. Useful to plot image in actual pixel size
    '''

    rows, cols =  imgs.shape[0],imgs.shape[1]

    subplot_shapes = np.array([list(map(np.shape, x)) for x in imgs])
    sp_height, sp_width, depth = np.max(np.max(subplot_shapes, axis=0), axis=0)

    title_pad = 30
    if titles is not None:
        pad_top = pad + title_pad
    else:
        pad_top = pad

    frame = np.zeros((rows*(sp_height+pad_top), cols*(sp_width+pad), depth ))

    for r in range(rows):
        for c in range(cols):
            img = imgs[r, c]
            h, w, _ = img.shape
            y0 = r * (sp_height+pad_top) + pad_top//2
            x0 = c * (sp_width+pad) + pad//2
            frame[y0:y0+h, x0:x0+w, :] = img

            if titles is not None:
                frame = cv2.putText(frame, titles[r, c], (x0, y0-title_pad//4), cv2.FONT_HERSHEY_COMPLEX, .5, (255,255,255))

    cv2.imshow(win_name, frame)
    cv2.waitKey(3)
# Define a function to show the image in an OpenCV Window
def show_image(name,img):
    img = cv2.resize(img,None,fx=0.3,fy=0.3,interpolation=cv2.INTER_AREA)
    cv2.imshow(name, img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # Try to convert the ROS Image message to a CV2 Image
    try:
        img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    #img = cv2.resize(img,None,fx=0.3,fy=0.3,interpolation=cv2.INTER_AREA)
    #show_image('img',img)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_blue= np.array([0,100,60])
    upper_blue = np.array([10,255,255])

    lower_green= np.array([40,100,60])
    upper_green = np.array([90,255,255])

    lower_red= np.array([100,100,60])
    upper_red = np.array([255,255,255])

    mask1 = cv2.inRange(hsv,lower_blue, upper_blue)

    result1 = cv2.bitwise_and(img,img,mask = mask1)

    #show_image('maskedblue',result1)
    mask2 = cv2.inRange(hsv,lower_green, upper_green)

    result2 = cv2.bitwise_and(img,img,mask = mask2)

    #show_image('maskedred',result2)
    mask3 = cv2.inRange(hsv,lower_red, upper_red)

    result3 = cv2.bitwise_and(img,img,mask = mask3)

    #show_image('maskedgreen',result3)


    lower_all= np.array([0,100,60])
    upper_all = np.array([180,255,255])

    mask = cv2.inRange(hsv,lower_all, upper_all)

    result = cv2.bitwise_and(img,img,mask = mask)

    #show_image('masked',result)

    img2 = np.pad(result.copy(), ((5,5), (5,5), (0,0)), 'minimum')
    #show_image("img2",img2)
    # call openCV with img2, it will set all the border pixels in our new pad with 0
    # now get rid of our border 

    #gray1 = cv2.cvtColor(img2,cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (9,9), 0)
    gray = cv2.blur(gray,(9,9))
    #show_image("gray",gray)
    #ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(gray,10,255,cv2.THRESH_BINARY)
    
    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
           cv2.THRESH_BINARY,31,0)
    #show_image("threshold",thresh)
    # find contours in the thresholded image
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    img = img2[1:-1,1:-1,:]


###B
    img2 = np.pad(result1.copy(), ((5,5), (5,5), (0,0)), 'minimum')
    #show_image("img2",img2)
    # call openCV with img2, it will set all the border pixels in our new pad with 0
    # now get rid of our border 

    #gray1 = cv2.cvtColor(img2,cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    #show_image("gray",gray)
    #gray = cv2.GaussianBlur(gray, (9,9), 0)
    gray = cv2.blur(gray,(9,9))
    #show_image("grayblurred",gray)
    #ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(gray,10,255,cv2.THRESH_BINARY)
    
    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
           cv2.THRESH_BINARY,31,0)
    #show_image("threshold",thresh)
    # find contours in the thresholded image
    cntsB = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cntsB = imutils.grab_contours(cntsB)

    imgBC = img2[1:-1,1:-1,:]

    for c in cntsB:
      # compute the center of the contour
      M = cv2.moments(c)
      if M["m00"]!=0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # draw the contour and center of the shape on the image
        cv2.drawContours(imgBC, [c], -1, (0, 255, 0), 2)
        cv2.circle(imgBC, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(imgBC, "center", (cX - 20, cY - 20),
              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        global detection_positions
        detection_positions = np.append(detection_positions, np.array([[UAVCurrentPosition.position.x,UAVCurrentPosition.position.y,3]]), axis=0)

    # Show the converted image
    #show_image("Contours",img)
###G
    img2 = np.pad(result2.copy(), ((5,5), (5,5), (0,0)), 'minimum')
    #show_image("img2",img2)
    # call openCV with img2, it will set all the border pixels in our new pad with 0
    # now get rid of our border 

    #gray1 = cv2.cvtColor(img2,cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (9,9), 0)
    gray = cv2.blur(gray,(9,9))
    #show_image("gray",gray)
    #ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(gray,10,255,cv2.THRESH_BINARY)
    
    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
           cv2.THRESH_BINARY,31,0)
    #show_image("threshold",thresh)
    # find contours in the thresholded image
    cntsG = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cntsG = imutils.grab_contours(cntsG)

    imgGC = img2[1:-1,1:-1,:]

    for c in cntsG:
      # compute the center of the contour
      M = cv2.moments(c)
      if M["m00"]!=0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # draw the contour and center of the shape on the image
        cv2.drawContours(imgGC, [c], -1, (0, 255, 0), 2)
        cv2.circle(imgGC, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(imgGC, "center", (cX - 20, cY - 20),
              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        global detection_positions
        detection_positions = np.append(detection_positions, np.array([[UAVCurrentPosition.position.x,UAVCurrentPosition.position.y,2]]), axis=0)

    # Show the converted image
    #show_image("Contours",img)
###R
    img2 = np.pad(result3.copy(), ((5,5), (5,5), (0,0)), 'minimum')
    #show_image("img2",img2)
    # call openCV with img2, it will set all the border pixels in our new pad with 0
    # now get rid of our border 

    #gray1 = cv2.cvtColor(img2,cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (9,9), 0)
    gray = cv2.blur(gray,(9,9))
    #show_image("gray",gray)
    #ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ret, thresh = cv2.threshold(gray,10,255,cv2.THRESH_BINARY)
    
    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
           cv2.THRESH_BINARY,31,0)
    #show_image("threshold",thresh)
    # find contours in the thresholded image
    cntsR = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cntsR = imutils.grab_contours(cntsR)

    imgRC = img2[1:-1,1:-1,:]
    # img is now set to the original dimensions, and the contours can be at the edge of the image
    #show_image("img",img)
    # loop over the contours
    for c in cntsR:
      # compute the center of the contour
      M = cv2.moments(c)
      if M["m00"]!=0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # draw the contour and center of the shape on the image
        cv2.drawContours(imgRC, [c], -1, (0, 255, 0), 2)
        cv2.circle(imgRC, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(imgRC, "center", (cX - 20, cY - 20),
              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        global detection_positions
        detection_positions = np.append(detection_positions, np.array([[UAVCurrentPosition.position.x,UAVCurrentPosition.position.y,1]]), axis=0)

    # Show the converted image
    
    #show_image("Contours",img)
    global UAVCurrentPosition
    for c in cnts:
      # compute the center of the contour
      M = cv2.moments(c)
      if M["m00"]!=0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # draw the contour and center of the shape on the image
        cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
        cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(img, "center", (cX - 20, cY - 20),
              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        global detection_positions
        #detection_positions = np.append(detection_positions, np.array([[UAVCurrentPosition.position.x,UAVCurrentPosition.position.y]]), axis=0)

    # Show the converted image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    show_image("Contours",img)
    #plt.scatter(detection_positions[:, 0], detection_positions[:, 1])
    #imgBC = cv2.cvtColor(imgBC, cv2.COLOR_BGR2RGB)
    #imgGC = cv2.cvtColor(imgGC, cv2.COLOR_BGR2RGB)
    #imgRC = cv2.cvtColor(imgRC, cv2.COLOR_BGR2RGB)
    #show_image("ContoursBlue",imgBC)
    #show_image("ContoursGreen",imgGC)
    #show_image("ContoursRed",imgRC)
    clicked5()
    #thresh = cv2.cvtColor(thresh,cv2.COLOR_GRAY2BGR)
    #imgs = np.array([[result1, result2, result3], [result, thresh, img]])
    #cvSubplot(imgs, pad=20 )
# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("uav/camera1/image_raw", Image, image_callback)
from Tkinter import *
 
window = Tk()
 
window.title("Detectcv")
 
window.geometry('1000x400')
 
lbl = Label(window, text="Number of points:0")
 
lbl.grid(column=0, row=0)
 
def clicked():
 
    
    np.save('data.npy', detection_positions) # save
    for postion in detection_positions:
      if postion[2] == 1: # equation of unit circle is x^2+y^2=1
          plt.scatter(postion[0],postion[1], color ='r')
      elif postion[2] == 2:
          plt.scatter(postion[0],postion[1], color ='g')
      else:
        plt.scatter(postion[0],postion[1], color ='b')

    #plt.scatter(detection_positions[:, 0], detection_positions[:, 1])
    
    lbl.configure(text=("Number of detections:"+str(detection_positions.shape[0])))
    plt.show()
 
btn = Button(window, text="Show points", command=clicked)
 
btn.grid(column=1, row=0)

def spawn_boxes(dimension=50,count=100,height=1):
    from gazebo_msgs.srv import SpawnModel
    from geometry_msgs.msg import Pose



    dimension=w3.get()
    count=w1.get()
    f_red = open('/home/marko/catkin_ws/src/larics_gazebo_worlds/models/red_box/model.sdf','r')
    sdff_red = f_red.read()

    f_green = open('/home/marko/catkin_ws/src/larics_gazebo_worlds/models/green_box/model.sdf','r')
    sdff_green = f_green.read()

    f_blue = open('/home/marko/catkin_ws/src/larics_gazebo_worlds/models/blue_box/model.sdf','r')
    sdff_blue = f_blue.read()
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    for i in range(count):
        color=np.random.randint(0, 3)+1
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
        global true_positions
        true_positions=np.append(true_positions, np.array([[x,y,color]]), axis=0)
def clicked2():

    spawn_boxes()
w1 = Scale(window, from_=0, to=200, orient=HORIZONTAL)
w1.set(100)
w1.grid(column=1, row=1)
btn2 = Button(window, text="Spawn boxes", command=clicked2)
 
btn2.grid(column=0, row=1)
def clicked3():
  fig = plt.figure()
  ax1 = fig.add_subplot(111)
  for postion in true_positions:
    if postion[2] == 1: # equation of unit circle is x^2+y^2=1
        ax1.scatter(postion[0],postion[1], color ='r',alpha=0.5, s=50,label='True positions red')
    elif postion[2] == 2:
        ax1.scatter(postion[0],postion[1], color ='g',alpha=0.5,s=50, label='True positions green')
    else:
      ax1.scatter(postion[0],postion[1], color ='b',alpha=0.5,s=50,label='True positions blue')

  for postion1 in detection_positions:
    if postion1[2] == 1: # equation of unit circle is x^2+y^2=1
        ax1.scatter(postion1[0],postion1[1], color ='r', s=5,label='Detected positions red')
    elif postion1[2] == 2:
        ax1.scatter(postion1[0],postion1[1], color ='g',s=5, label='Detected positions green')
    else:
      ax1.scatter(postion1[0],postion1[1], color ='b',s=5 ,label='Detected positions blue')
  ax1.plot(positions[:,0], positions[:,1],color ='k', linestyle='-',linewidth=100,alpha=0.2)
  #plt.scatter(detection_positions[:, 0], detection_positions[:, 1])
  lbl.configure(text=("Number of detections:"+str(detection_positions.shape[0])))
  plt.show()
 
btn2 = Button(window, text="Draw True and detected boxes", command=clicked3)
 
btn2.grid(column=2, row=1)
def clicked4():
    global detection_positions
    detection_positions = np.empty((0,3), int)
    global positions
    positions=np.empty((0,2), int)

 
btn3 = Button(window, text="Delete detections and uav_pos", command=clicked4)
 
btn3.grid(column=0, row=2)

lbl1 = Label(window, text="Detected points/total points=0%")
 
lbl1.grid(column=0, row=3)

lbl4 = Label(window, text="Detected 0 boxes")
 
lbl4.grid(column=2, row=3)

def clicked5():
  detected=0
  total_points=true_positions.shape[0]
  detection_radius=w2.get()
  for postion in true_positions:
    for postion1 in detection_positions:
      #print("colortrue="+str(postion[2])+"colordetected="+str(postion1[2]))
      if math.sqrt( ((postion[0]-postion1[0])**2)+((postion[1]-postion1[1])**2) )<detection_radius and int(postion[2])==int(postion1[2]):
        detected+=1
        break
  if(total_points!=0):
    lbl1.configure(text=("Detected boxes/total boxes="+str(round(100*(detected/total_points),2))+"%"))

  lbl4.configure(text=("Detected: "+str(detected)+" boxes"))
  lbl.configure(text=("Number of detections:"+str(detection_positions.shape[0])))
 
btn4 = Button(window, text="Calculate percentage of detection", command=clicked5)
 
btn4.grid(column=1, row=3)
del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # model spawner
def clicked6():
  for i in range(w1.get()):
    ref=del_model_prox("box"+str(i)) # Remove from Gazebo
  global true_positions
  true_positions = np.empty((0,3), int)
def del_model( modelName ): # FIXME: Freezes Python, DO NOT USE!
  """ Remove the model with 'modelName' from the Gazebo scene """
  # delete_model : gazebo_msgs/DeleteModel
  del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # model spawner
  # rospy.wait_for_service('gazebo/delete_model') # Wait for the model loader to be ready 
  # FREEZES EITHER WAY
  ref=del_model_prox(modelName) # Remove from Gazebo

 
btn5 = Button(window, text="Delete boxes", command=clicked6)
 
btn5.grid(column=0, row=4)

w2 = Scale(window, from_=1, to=30, orient=HORIZONTAL)
w2.grid(column=2, row=4)
w2.set(4)
w3 = Scale(window, from_=1, to=500, orient=HORIZONTAL)
lbl3 = Label(window, text="Space dimension:")
 
lbl3.grid(column=1, row=2)
w3.grid(column=2, row=2)
w3.set(50)
lbl2 = Label(window, text="Detection radius:")
 
lbl2.grid(column=1, row=4)


# This function is called periodically from FuncAnimation
def animate(i):
    global ax1
    for postion in true_positions:
      if postion[2] == 1: # equation of unit circle is x^2+y^2=1
          ax1.scatter(postion[0],postion[1], color ='r',alpha=0.5, s=50,label='True positions red')
      elif postion[2] == 2:
          ax1.scatter(postion[0],postion[1], color ='g',alpha=0.5,s=50, label='True positions green')
      else:
        ax1.scatter(postion[0],postion[1], color ='b',alpha=0.5,s=50,label='True positions blue')

    for postion1 in detection_positions:
      if postion1[2] == 1: # equation of unit circle is x^2+y^2=1
          ax1.scatter(postion1[0],postion1[1], color ='r', s=5,label='Detected positions red')
      elif postion1[2] == 2:
          ax1.scatter(postion1[0],postion1[1], color ='g',s=5, label='Detected positions green')
      else:
        ax1.scatter(postion1[0],postion1[1], color ='b',s=5 ,label='Detected positions blue')
    ax1.plot(positions[:,0], positions[:,1],color ='k', linestyle='-',linewidth=10,alpha=0.2)

    # Draw x and y lists
    #ax1.clear()


    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')




#line, = ax1.plot(positions[:,0], positions[:,1],color ='k', linestyle='-',linewidth=10,alpha=0.2)


def animate(i):
    line.set_xdata(positions[:,0])  # update the data.
    line.set_ydata(positions[:,1])  # update the data.
    ax1.relim()
    # update ax.viewLim using the new dataLim
    ax1.autoscale_view()
    return line,


#ani = animation.FuncAnimation(fig, animate, interval=20)
#plt.show()

window.mainloop()

#new_detection_positions = np.load('data.npy') # load
# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()