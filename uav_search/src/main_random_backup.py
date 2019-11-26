import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.widgets import Slider, Button, RadioButtons,Rectangle
#x = np.random.uniform(0, 1, 10)
#y = np.random.uniform(0, 1, 10)
#right 1,0
#down 0,-1
#left -1,0
#up 0,1
#ccw rotation za 90
#x'=a+b-y
#y'=x+b-a

xs=[]
ys=[]

fig = plt.figure()
ax = fig.add_subplot(111)
axis_color = 'lightgoldenrodyellow'
#ax.set_xlim([-100, 110])
#ax.set_ylim([-100,110])

length=40
spacing=20
dimension=100
sigma=1

directions=[[1,0],[0,-1],[-1,0],[0,1]]
startinglabel="uniform"

txt=plt.gcf().text(0.1, 0.9, 'Total distance='+"{0:.2f}".format(0)+"m", fontsize=14)
def updatePlot():
    points = generate()
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
    #if startinglabel == "uniform":
    points = generatepoints(int(amp_slider.val), amp_slider1.val,int(amp_slider2.val),amp_slider3.val,startinglabel)
    #else:
    #    points = generatepointsreverse(int(amp_slider.val), amp_slider1.val)
    return points
def generatepoints(length,spacing,dimension,sigma,startinglabel="uniform"):
    if (spacing < 10e-7):
        spacing = 10e-3
    # if spacing>dimension*(3/5):
    #     spacing=dimension*(3/5)
    start = np.array([[0, 0]])
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
        points=np.vstack((points,np.array([[x,y]])))
        distance_travelled += np.linalg.norm(np.array([a,b])-np.array([x,y]))
        xs.append(x)

        ys.append(y)

    message='Total distance='+"{0:.2f}".format(distance_travelled)+"m"
    txt.set_text(message)

    return points

def generatepointsreverse(length,spacing):

    if(spacing<10e-7):
        spacing=10e-3
    length=length/spacing
    start = np.array([[0, 0]])
    points = np.array(start)
    for i in (range(1,int(length))):
        a=points[i-1][0]
        b=points[i-1][1]
        direction = int(np.floor(np.random.randint(0, 4)))
        x = points[i - 1][0] + np.array(directions[direction][0]) * ((length-i) * spacing)
        y = points[i - 1][1] + np.array(directions[direction][1]) * ((length-i) * spacing)

        xnew=a+b-y
        ynew=x+b-a
        xs.append(xnew)
        ys.append(ynew)
        points=np.vstack((points,np.array([[xnew,ynew]])))
    return points
# Plot non-ordered points
points=generatepoints(length,spacing,dimension,sigma)
[line] = ax.plot(points[:,0], points[:,1], marker="o", markerfacecolor="r", linestyle='-')




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
amp_slider2 = Slider(amp_slider_ax2, 'Search space dimension', 0, 200, valinit=amp_2,valfmt= "%.2f m")
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
regenerate_button_ax = fig.add_axes([0.8, 0.0, 0.1, 0.04])
regenerate_button = Button(regenerate_button_ax, 'Regenerate', color=axis_color, hovercolor='0.975')
def regenerate_button_on_clicked(mouse_event):
    updatePlot()
regenerate_button.on_clicked(regenerate_button_on_clicked)

color_radios_ax = fig.add_axes([0.025, 0.5, 0.15, 0.15], facecolor=axis_color)
color_radios = RadioButtons(color_radios_ax, ('uniform', 'normal','normal_with_turn',"normal_with_turn_and_scale","levy"), active=0)
def color_radios_on_clicked(label):
    global startinglabel
    startinglabel=label
    updatePlot()
color_radios.on_clicked(color_radios_on_clicked)


ax.add_patch(Rectangle((0, 0), int(amp_slider2.val), int(amp_slider2.val),alpha=0.2))
plt.show()