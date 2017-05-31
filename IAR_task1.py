
# File: IAR_task1.py
# Date last edited: 05.10.2016
# Intelligent Autonomous Robotics Task 1
# Stefanos Loizou (s1217795) and Clara Tump (s1679058)

from khepfun import *
import time

# returns true if the value of n is between the value of h 
# and the value of l
def between(h,l,n):
 return n < h and n>l

if __name__ == "__main__" :
 s = open_connection()
 set_counts(s, 0, 0)
 speed = 8
 try:
  # keeps looping until key is pressed
  while (1):
   IRreadings = read_IR(s)
   # dm,dl and dr are the averages of the IR readings by the
   # two left, the two front and the two right IR sensors
   dl = (IRreadings[0] + IRreadings[1]) /2
   dm = (IRreadings[2] + IRreadings[3]) /2 
   dr = (IRreadings[4] + IRreadings[5]) /2

   # thresh_..._high is the upper threshold for the IR readings on 
   # all three sides (collision if this threshold is exceeded). 
   # The left and right side also have a thresh_..._low
   # which means the robot is too far from the wall it is following
   thresh_left_high =  180
   thresh_left_low =  50
   thresh_mid_high =  100
   thresh_right_high = 225
   thresh_right_low =  96

   # The target IR readings when following a wall
   left_target = 160
   right_target = 200

   # When the high thresholds are exceeded: collision danger
   tl = dl > 200
   tm = dm > 127
   tr = dr > 245


   if (tl or tm or tr ):
    # if collision danger: turn on the spot, always to the right
    mult = -1
    set_speeds(s,speed * mult, mult * -speed)

   elif (between(thresh_left_high, thresh_left_low,dl)):
    # if there is a wall on the left, follow it
    y = (dl - left_target) / 10
    set_speeds(s, speed + (y/2), speed - (y/2))

   elif (between(thresh_right_high,thresh_right_low,dr)):
    # if there is a wall on the right, follow it
    y = (dr - right_target) / 10
    set_speeds(s,speed - (y/2), speed + (y/2))

   else:
    # if there is nothing around, just go straight
    set_speeds(s,speed,speed)

 except KeyboardInterrupt:
  set_speeds(s,0,0)


