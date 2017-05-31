#####################################################################
# File: IAR_task2.py
# Date last edited: 19.10.2016
# Intelligent Autonomous Robotics Task 2
# Stefanos Loizou (s1217795) and Clara Tump (s1679058)
#####################################################################
# The robot moves around a maze without bumping into obstacles and
# following long walls. 
#####################################################################

from __future__ import division
from khepfun import *
import time
from math import *
from slam import *

#the diameter of the robot - the distance between the left and right wheel
diam = 55

delta_right_prev = 0
delta_left_prev = 0

# makes the speed of the robot proportional to the angle it has to turn
# this causes smooth turns
def speed_factor(x):
    if x == 0:
        return 1
    else:
        return ((x - 180) % 180)/180

# returns the euclidean distance between point(x1,y1) and point(x2,y2)
def distanceSqEucl(x1, y1, x2, y2):
    diffX = x1 - x2;
    diffY = y1 - y2;
    return (diffX * diffX + diffY * diffY)

# says whether the goal is on the left or on the right
def direction( x1,  y1,  x2,  y2, x3,  y3):
    d = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1))
    return d

# get the angle between vectors v1 and v2
def get_angle(v1,v2):
    x1 = v1[0]
    y1 = v1[1]
    x2 = v2[0]
    y2 = v2[1]
    x3 = 0
    y3 = 0
    d = direction(x3, y3, x2, y2, x1, y1)
    d1d3 = distanceSqEucl(x1, y1, x3, y3)
    d2d3 = distanceSqEucl(x2, y2, x3, y3)
    d1d2 = distanceSqEucl(x1, y1, x2, y2)
    cosA = (d1d3 + d2d3 - d1d2)    / (2 * math.sqrt(d1d3 * d2d3));
    angleA = math.acos(cosA);
    if (d > 0) :
        angleA = (2.*math.pi - angleA)


    return (-math.degrees(angleA) + 180)  % 360


def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return sqrt(dotproduct(v, v))

def angle(v1, v2):
 dp = dotproduct(v1, v2) / (length(v1) * length(v2))
   return acos(dp)

def between(h,l,n):
 return n < h  and n>l

# add the vector of the movement of the current movement to 
# the previous pose
def update_pose(pose, dl, dr):
 new_x = ((dr + dl)/2) * cos(pose[2] + ((dr - dl) / diam))
 new_y = ((dr + dl)/2) * sin(pose[2] + ((dr - dl) / diam))
 new_theta = (dr - dl) / (diam)
 pose[0] = pose[0] + new_x
 pose[1] = pose[1] + new_y
 pose[2] = pose[2] + new_theta
 return pose

# reset the boredom counts so that when a new wall is encountered
# the robot is not immediately bored
def reset_boredoms(boredom_left, boredom_right):
 if(boredom_left > 0 or boredom_right > 0):
 boredom_left = 0
 boredom_right = 0
 return boredom_left, boredom_right
 

if __name__ == "__main__" :
 s = open_connection()
 ma = SLAM()
 r = Robot()
 m = ma.get_map()
 
 sx = 2000
 sy = 2000
 route = []
 thresh_left_high =  180
 thresh_left_low =  50
 thresh_mid_high =  100
 thresh_right_high = 225
 thresh_right_low =  96
 bore_limit = 400
 left_target = 160
 right_target = 200

 set_counts(s, 0, 0)
 pose = [sx,sy,0]
 speed = 8
 boredom_left = 0
 boredom_right = 0
 keypressed = None

 try:
  while (1):
   IRreadings = read_IR(s)
   Oreadings = read_counts(s)
   delta_left = Oreadings[0]/12
   delta_right = Oreadings[1]/12

   delta_right = delta_right - delta_right_prev
   delta_left = delta_left - delta_left_prev

   pose = update_pose(pose, delta_left, delta_right)
   delta_left_prev += delta_left
   delta_right_prev += delta_right
   
   m = ma.get_map()
   r.x = pose[0]
   r.y = pose[1]
   r.direction = pose[2]
   route.append((r.x,r.y,r.direction))

   r.draw(m)
   m = cv2.resize(m,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_CUBIC)
   cv2.imshow("map", m )

   dm = (IRreadings[2] + IRreadings[3]) /2 
   dl = (IRreadings[0] + IRreadings[1]) /2
   dr = (IRreadings[4] + IRreadings[5]) /2
   diff = IRreadings[2] - IRreadings[3]

   tl = dl > 200
   tm = dm > 127
   tr = dr > 245
   
   # if an obstacle is too close, move on the spot to the left
   if (tl or tm or tr ):
    boredom_left, boredom_right = reset_boredoms(boredom_left, boredom_right)
    mult = -1
    set_speeds(s,speed * mult, mult * -speed)

   # if a key is pressed, get back to the starting position
   elif (keypressed):
    v2 = r.get_dir_vec()
    v1 = [(sx-pose[0]),(sy-pose[1])]
    x = get_angle(v1,v2)
    sf = speed_factor(x)

    if abs(v1[0]) < 20 and abs(v1[1]) < 20:
     set_speeds(s,0,0)
    elif (x>180):
     print 'l'
     set_speeds(s,speed -(speed*sf),speed + (speed*sf))
    else:
     print "r"
     set_speeds(s,speed +(speed*sf) ,speed-(speed*sf))
     
   # follow a wall on the left
   elif (between(thresh_left_high,thresh_left_low,dl) ):
    if(boredom_left > bore_limit):
     # the robot is probably following an 'infinite wall'
     set_speeds(s,speed,-speed)
     time.sleep(0.2)

    y = (dl - left_target) / 10
    set_speeds(s,speed +(y/2) ,speed - (y/2))
    boredom_left = boredom_left + 1

   # follow a wall on the right
   elif (between(thresh_right_high,thresh_right_low,dr)):
    if(boredom_right > bore_limit):
    # the robot is probably following an 'infinite wall'
     set_speeds(s,-speed,speed)
     time.sleep(0.2)
    y = (dr - right_target) / 10
    set_speeds(s,speed - (y/2) ,speed + (y/2))
    boredom_right = boredom_right + 1
    print boredom_right
   else:
    boredom_left, boredom_right = reset_boredoms(boredom_left, boredom_right)
    set_speeds(s,speed,speed)

   k = cv2.waitKey(10) & 0xFF
   if k == ord('c'):
    keypressed = not keypressed   

 except KeyboardInterrupt:
  set_speeds(s,0,0)
