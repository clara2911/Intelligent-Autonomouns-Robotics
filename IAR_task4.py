#####################################################################
# File: IAR_task4.py
# Date last edited: 07.11.2016
# Intelligent Autonomous Robotics Task 4
# Stefanos Loizou (s1217795) and Clara Tump (s1679058)
#####################################################################
# The robot explores a maze (without bumping into obstacles and 
# following long walls). When a key is pressed (food location) it moves
# back to its home position, and then starts exploring the maze again.
#####################################################################


from __future__ import division
from khepfun import *

import time
from math import *
from slam import *


diam = 55
delta_right_prev = 0
delta_left_prev = 0


def speed_factor(x):
    if x == 0:
        return 1
    else:
        return ((x - 180) % 180)/180

def distanceSqEucl(x1, y1, x2, y2):

    diffX = x1 - x2;
    diffY = y1 - y2;
    return (diffX * diffX + diffY * diffY)


def direction( x1,  y1,  x2,  y2, x3,  y3):

    d = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1))
    return d


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
  	# print dp
  	return acos(dp)

def between(h,l,n):
	return n < h  and n>l

def update_pose(pose, dl, dr,diam=55):
	new_x = ((dr + dl)/2) * cos(pose[2] + ((dr - dl) / diam))
	new_y = ((dr + dl)/2) * sin(pose[2] + ((dr - dl) / diam))
	new_theta = (dr - dl) / (diam)
	pose[0] = pose[0] + new_x                
	pose[1] = pose[1] + new_y
	pose[2] = pose[2] - new_theta
	return pose

def get_particles(pose,dl,dr):
	pose = update_pose(pose, dl, dr)
	particles = np.matlib.repmat(pose,100,1)
	std = [[1,0,0],[0,1,0],[0,0,1]]
	noise = np.random.multivariate_normal([0,0,0],std, 100)
	y = multivariate_normal.pdf(noise, mean=[0,0,0] , cov=std)
	return particles + noise , y


def reset_boredoms(boredom_left, boredom_right):
	if(boredom_left > 0 or boredom_right > 0):
		print("RESET BOREDOMS")
	boredom_left = 0
	boredom_right = 0
	return boredom_left, boredom_right

def go_home(r,s,speed,img,int_goal):
	bool_final = False
	v2 = r.get_dir_vec()
	if(reached(r, int_goal)):
		int_goal, bool_final = Astar_find_int_goal(r.x,r.y,[sx,sy],img):
	v1 = [(int_goal[0]-pose[0]),(int_goal[0]-pose[1])]
	x = get_angle(v1,v2)
	sf = speed_factor(x)
	if abs(v1[0]) < 20 and abs(v1[1]) < 20:
		flash_leds()
		if bool_final:
			keypressed = None
	elif (x>180):
		set_speeds(s,speed -(speed*sf),speed + (speed*sf))
	else:
		set_speeds(s,speed +(speed*sf) ,speed-(speed*sf))
	return int_goal

def reached(r, int_goal):
	# if x_error + y_error is smaller than 20, the goal is reached
	if(abs(r.x-int_goal[0]) + abs(r.y-int_goal[1])<20):
		return True
	else:
		return False

def flash_leds():
	i = 0
	for i in range(0,6):
		set_led(s,0,2)
		set_led(s,1,2)
		sleep(0.2)

def mark_foodlocation(pose):
	foodlocations.append([pose[0], pose[1]])
	return foodlocations

def go_to_food(i,r,v1,s,speed, img, int_goal):
	bool_final = False
	v2 = r.get_dir_vec()
	if(reached(int_goal)):
		int_goal = Astar_find_int_goal(r.x,r.y,[sx,sy],img):
	v1 = [(int_goal[i][0]-pose[0]),(int_goal[i][1]-pose[1])]
	x = get_angle(v1,v2)
	sf = speed_factor(x)
	if abs(v1[0]) < 20 and abs(v1[1]) < 20:
		if bool_final:
			keypressed = None
	elif (x>180):
		set_speeds(s,speed -(speed*sf),speed + (speed*sf))
	else:
		set_speeds(s,speed +(speed*sf) ,speed-(speed*sf))
	return int_goal	

def Astar_find_int_goal(rx,ry,goal_coords,SLAM_map):
	coords_int_goal = find_Astarpath(robot_coords, goal_coords, SLAM_map)
	return coords_int_goal


# finds the shortest path to the goal and returns an intermediate goal
# to move towards
def find_Astarpath(robot_coords, goal_coords, img):
	Astar_list = []
	path = []
	current_coords = robot_coords
	iterations = 0
	final_goal = False
	while current_coords != goal_coords:
		neighbors_coords = find_neighbors(current_coords, img)
		neighbors_info = attach_g_plus_h(neighbors_coords,iterations, goal_coords)
		for neighbor_info in neighbors_info:
			Astar_list.append(neighbor_info)
		#sort list based on g_plus_h values
		sorted_Astar_list = sorted(Astar_list, key=itemgetter(1), reverse=True)
		Astar_list.remove(sorted_Astar_list[-1])
		current_coords = sorted_Astar_list[-1][0]
		path.append(current_coords)
		iterations = iterations+1
		if iterations > 1000:
			break
	#return the coords of the best intermediate goal
	if len(path) > 10:
		return path[10], final_goal
	else:
		final_goal = True
		return path[-1], final_goal

def find_neighbors(inputcoords, img):
	neighbors_coords = []
	[x,y] = inputcoords
	pixels = [[x-1,y-1], [x-1,y], [x-1,y+1], [x,y-1], [x,y+1], [x+1,y-1], [x+1,y], [x+1,y+1]]
	for pixel in pixels:
		# when it is black or it is white?
		if not all(img[pixel[0]][pixel[1]] == [255,255,255]):
			neighbors_coords.append(pixel)
	return neighbors_coords


def attach_g_plus_h(neighbors_coords, iterationNo, goal_coords):
	neighbors_info = []
	for neighbor in neighbors_coords:
			g = iterationNo
			h = ((goal_coords[0] - neighbor[0])**2 + (goal_coords[1] - neighbor[1])**2)**(0.5)
			gplush = g+h
			neighbor_info = [neighbor, gplush]
			neighbors_info.append(neighbor_info)
	return neighbors_info	

def move_to(i,r,s,speed):
	v2 = r.get_dir_vec()
	v1 = [(foodlocations[i][0]-pose[0]),(foodlocations[i][1]-pose[1])]
	x = get_angle(v1,v2)
	sf = speed_factor(x)
	if abs(v1[0]) < 20 and abs(v1[1]) < 20:
		keypressed = None
	elif (x>180):
		set_speeds(s,speed -(speed*sf),speed + (speed*sf))
	else:
		set_speeds(s,speed +(speed*sf) ,speed-(speed*sf))

if __name__ == "__main__" :
	# if this file is run as a script, it will run through some function calls
	s = open_connection()
	set_speeds(s,0,0)

	SCALE = 0.5
	realwidth = 1400 
	realheight = 760  
	im = cv2.imread('arena_16_shade.jpg')
	
	wfactor = SCALE *realheight/im.shape[0]
	hfactor = SCALE * realwidth/im.shape[1]
	im = cv2.resize(im,None,fx=wfactor, fy=hfactor, interpolation = cv2.INTER_CUBIC)
	slam2 = SLAM(im,scale = SCALE)

	
	
	r = Robot(scale = SCALE)
	m = slam2.get_map()
	m = r.draw(m)
	cv2.imshow("map", m )
	cv2.waitKey(0)
	
	sx = 680 * SCALE
	sy = 570 * SCALE
	route = []
	thresh_left_high =  180
	thresh_left_low =  50


	thresh_mid_high =  100
	thresh_right_high = 225
	thresh_right_low =  96
	bore_limit = 400
	left_target = 160
	right_target = 200

	#print "\nResetting wheel encoders!"
	set_counts(s, 0, 0)

	### GENERATE THE STARTING POSES
	pose = [sx,sy,0]
	poses = []
	pose_probabilites =  []
	## Generate 20 poses in equidistant angles with equal probability
	n = 10
	y = 2*math.pi/n
	for x in range(0,n):
		p = pose[:]
		p[2] = y * x
		poses.append(p)
		pose_probabilites.append(1/n)
	print poses,pose_probabilites

	pose = poses[0]
	pose[2] = -math.pi/2
	r.x = pose[0]
	r.y = pose[1]
	r.direction = pose[2]
	poses = [pose]
	print pose

	m = slam2.get_map()
	m = r.draw(m)
	cv2.imshow("map", m )
	print "STARTING POSSITION SET"
	cv2.waitKey(0)

	speed =8
	boredom_left = 0
	boredom_right = 0
	keypressed = None
	SET_SPEED_FN = set_speeds
	int_goal = [0,0]
	foodlocations = []

	def STOP (s,x,y):
		set_speeds(s,0,0)
	
	set_speeds_c = SET_SPEED_FN
	try:
		while (1):
			#Get readings

			IRreadings = read_IR(s)
			Oreadings = read_counts(s)
			delta_left = (Oreadings[0]/12) * SCALE
			delta_right = (Oreadings[1]/12) * SCALE

			delta_right = delta_right - delta_right_prev
			delta_left = delta_left - delta_left_prev

			#Update possible poses given the odometry data
			poses = [update_pose(pose, delta_left, delta_right,r.radius*2) for pose in poses]
			poses, pose_probabilites = do2(poses,pose_probabilites,slam2,r,IRreadings)
			print "ENDING", poses,pose_probabilites

			#print pose
			delta_left_prev += delta_left
			delta_right_prev += delta_right
			#print pose
			# time.sleep(0.00)
			m = slam2.get_map().copy()
			pose = poses[0]
			# pose = update_pose(pose, delta_left, delta_right,r.diam * 2)
			r.x = pose[0]
			r.y = pose[1]
			r.direction = pose[2]
			slam2.set_visited(r)

			route.append((r.x,r.y,r.direction))

			m= r.draw(m)
			cv2.imshow("map", m )
			cv2.imshow("visited",slam2.visited_map)

			dm = (IRreadings[2] + IRreadings[3]) /2 
			dl = (IRreadings[0] + IRreadings[1]) /2
			dr = (IRreadings[4] + IRreadings[5]) /2
			diff = IRreadings[2] - IRreadings[3]

			tl = dl > 200
			tm = dm > 127
			tr = dr > 245

			if (tl or tm or tr ):
				boredom_left, boredom_right = reset_boredoms(boredom_left, boredom_right)
				mult = -1 #if dr > dl + 50 else 1
				#print mult, dm,dl,dr
				set_speeds_c(s,speed * mult, mult * -speed)

			elif (keypressed):
				mark_foodlocation(pose, foodlocations)
				int_goal = go_home(r,s,speed, slam2.mapas)

			elif(len(foodlocations) > 0):
				# go to the i'th food location. We assume now we only know 1
				i=0
				go_to_food(i,r,v1,s,speed, slam.mapas, int_goal)
			elif (between(thresh_left_high,thresh_left_low,dl) ):
				if(boredom_left > bore_limit):
					set_speeds(s,speed,-speed)
					time.sleep(0.2)

				y = (dl - left_target) / 10
				set_speeds(s,speed +(y/2) ,speed - (y/2))
				boredom_left = boredom_left + 1
			elif (between(thresh_left_high,thresh_left_low,dl) ):
				# print("F LEFT WALL")
				if(boredom_left > bore_limit):
					# print("BORED LEFT WALL")
					set_speeds_c(s,speed,-speed)
					time.sleep(0.2)

				y = (dl - left_target) / 10
				set_speeds_c(s,speed +(y/2) ,speed - (y/2))
				boredom_left = boredom_left + 1

			elif (between(thresh_right_high,thresh_right_low,dr)):
				if(boredom_right > bore_limit):
					set_speeds_c(s,-speed,speed)
					time.sleep(0.2)
				y = (dr - right_target) / 10
				set_speeds_c(s,speed - (y/2) ,speed + (y/2))
				boredom_right = boredom_right + 1
				print boredom_right
			else:
				boredom_left, boredom_right = reset_boredoms(boredom_left, boredom_right)
				set_speeds_c(s,speed,speed)



			k = cv2.waitKey(1) & 0xFF
			if k == ord('c'):
				keypressed = not keypressed
			# if k == ord(' '):
			# 	set_speeds_c = SET_SPEED_FN
			# else:
			# 	set_speeds_c = STOP
			

	except KeyboardInterrupt:
		set_speeds_c(s,0,0)
