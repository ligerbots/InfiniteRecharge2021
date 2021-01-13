#!/usr/bin/env python



import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.ticker import MaxNLocator

import random

def feetToMeter(feet):
    return(0.3048*feet)
def meterToFeet(meter):
    return(3.28084*meter)

field_width=20
field_height=22

foreground_color = 'white'
plt.rcParams['text.color'] = foreground_color
plt.rcParams['axes.labelcolor'] = foreground_color
plt.rcParams['xtick.color'] = foreground_color
plt.rcParams['ytick.color'] = foreground_color

plt.rcParams['axes.facecolor'] = 'black'
plt.rcParams.update({'font.size': 22})

fig = plt.figure(figsize=(field_width, field_height))
ax = fig.add_subplot(1, 1, 1)

ax.spines['right'].set_position(('axes', 0))
ax.spines['top'].set_position(('axes', 0))

ax.spines['left'].set_color('none')
ax.spines['bottom'].set_color('none')

ax.xaxis.set_ticks_position('top')
ax.yaxis.set_ticks_position('right')

ax.grid(linestyle='-', linewidth='0.5')

plt.axis([0, field_width, 0, field_height]) 
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

ax.set_xticks(np.arange(1, field_width, 1))# do not draw 0
ax.set_yticks(np.arange(1, field_height, 1))

def rect(x1,y1,x2,y2,style):
    plt.plot([x1,x2],[y1,y1],style)
    plt.plot([x1,x2],[y2,y2],style)
    plt.plot([x1,x1],[y1,y2],style)
    plt.plot([x2,x2],[y1,y2],style)
    
def line(x1,y1,x2,y2):
    plt.plot([x1,x2],[y1,y2],style)

def robotCurve(inner_radius,color):
    curve=[]
    start=np.array((10,3))
    curve=[start]

    turnentryexit_topright=start+np.array((0-robot_width/2,10+robot_height/2))
    curve.append(turnentryexit_topright+np.array((robot_width/2,-robot_height/2)))
    
    turncenter=turnentryexit_topright+np.array((-inner_radius,inner_radius))
    radius=inner_radius+robot_width/2
    
    plt.plot([turncenter[0]],[turncenter[1]], 'ro',color=color)

    for theta in np.arange(0.0, math.pi*1.5, 0.05):
        curve.append(turncenter+np.array((radius*math.cos(theta),radius*math.sin(theta))))
    curve.append(turnentryexit_topright+np.array((robot_height/2,-robot_width/2)))
    curve.append(curve[-1]+np.array((2,0)))

    transposed=np.array(curve).T
    plt.plot(transposed[0],transposed[1],color=color)
    
    drawRobot(*curve[0])
    drawRobot(*curve[-1],True)

robot_width=27/12
robot_height=28/12
def drawRobot(x,y,isVertical=True):
    if(isVertical):
        rect(x-robot_width/2,y-robot_height/2,x+robot_width/2,y+robot_height/2,"b-")
    else:
        rect(x-robot_height/2,y-robot_width/2,x+robot_height/2,y+robot_width/2,"b-")

cmap=plt.cm.get_cmap("hsv", 5)  

robotCurve(0,cmap(0))
robotCurve(1/2,cmap(1))
robotCurve(1,cmap(2))
robotCurve(2,cmap(3))

plt.savefig("fieldmap.png", bbox_inches='tight', pad_inches=0, dpi=80, transparent=True)

print("Field width: "+str(feetToMeter(field_width))+" height: "+str(feetToMeter(field_height)))
print("Robot width: "+str(feetToMeter(robot_width))+" height: "+str(feetToMeter(robot_height)))

plt.show()
