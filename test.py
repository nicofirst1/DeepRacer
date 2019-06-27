import math
from random import randint

import matplotlib

from reward_f import reward_function, HyperParams
import ast
import matplotlib.pyplot as plt
import numpy as np

with open("tokyo1.txt","r") as file:
    params=eval(file.read())


def get_slope(x,y,theta):
    """Return the slope of the line given a point and an angle"""
    m=math.tan(math.radians(theta))
    c=y-m*x
    return m,c

wp=params[0]
params=params[1:]

hp=HyperParams()
plt.ion()

plt.show()

start=randint(0,len(params))

for p in params[start:]:

    p['waypoints']=wp

    
    # plotting wps
    plt.scatter(*zip(*wp),  alpha=0.2, color="blue",s=2)
    plt.scatter(p['x'],p['y'],  alpha=0.5, color="red")
    waypoints = p["waypoints"]
    wp_idx = p["closest_waypoints"]

    nxt_wps=[]

    range_max = min(wp_idx[1] + hp.FUTURE_WAYPOINT_NUMBER, len(waypoints))
    for idx in range(wp_idx[1], range_max):
        nxt_wps.append(waypoints[idx])

    # if the waypoints start again from zero get the followings
    if wp_idx[1] + hp.FUTURE_WAYPOINT_NUMBER > len(waypoints):
        range_max = hp.FUTURE_WAYPOINT_NUMBER - (len(waypoints) - wp_idx[1])
        for idx in range(range_max):
            nxt_wps.append(waypoints[idx])



    curvature_wps=nxt_wps[:hp.CURVATURE_WAYPOINT_NUMBER]
    position_wps=nxt_wps[:hp.POSITION_WAYPOINT_NUMBER]
    steer_wps=nxt_wps[:hp.STEERING_WAYPOINT_NUMBER]

    plt.scatter(*zip(*curvature_wps),  alpha=0.3, color="green")
    plt.scatter(*zip(*position_wps),  alpha=0.3, color="orange")
    plt.scatter(*zip(*steer_wps),  alpha=0.3, color="pink")


    m,c=get_slope(p['x'],p['y'],p['heading'])
    x=np.linspace(p['x']-1,p['x']+1)
    plt.plot(x, m*x+c, '-r')
    m, c = get_slope(p['x'], p['y'], p['steering_angle'])
    x = np.linspace(p['x'] - 1, p['x'] + 1)
    plt.plot(x, m * x + c, '-g')

    plt.draw()
    plt.pause(0.1)
    r=reward_function(p)

    print("\n\n")
    plt.clf()





