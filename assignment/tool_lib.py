import time
import math

class Flag:
	def __init__(self, start_flag, end_flag, start_lap, end_lap, dist_flag, xR, yR, distance_travelled):
		self.start_flag = start_flag 					#bool: value to determine if the time has already been taken
		self.end_flag = end_flag						#bool: value to determine if the time has already been taken
		self.start_lap = start_lap						#time: variable to store the instant of the start of the lap
		self.end_lap = end_lap 							#time: variable to store the instant of the end of the lap
		self.dist_flag = dist_flag						#bool: flag to check the correctness of the distance computation
		self.xR = xR									#float: x coordinate of the robot position
		self.yR = yR									#float: y coordinate of the robot position
		self.distance_travelled = distance_travelled	#float: total distance travelled by the robot


flag = Flag(False, False, 0, 0, False, 0, 0, 0)

def distance_computation(actualX, actualY):
	flag.dist_flag = True

	xIn = -8
	yIn = -4
	
	flag.distance_travelled += math.sqrt(pow(actualX - xIn, 2) + pow(actualY - yIn, 2))
	
	xIn = actualX
	yIn = actualY

def time_computation(xPass, yPass):
	flag.xR = xPass
	flag.yR = yPass
	distance_computation(flag.xR, flag.yR)
	
	if -9.00 <= flag.xR <= -7.00 and -3.70 <= flag.yR <= -3.30:
		if not(flag.start_flag) and not(flag.end_flag):
			flag.start_lap = time.time()
			flag.start_flag = True
			f = open("stats/lap_time_assignment.txt", 'a')
			f.write("Str [s]: " + str(flag.start_lap) + "\n")
			f.close()
	if -7.60 <= flag.xR <= -7.40 and -5.00 <= flag.yR <= -3.00:
		if not(flag.end_flag) and flag.start_flag:
			flag.end_lap = time.time()
			flag.end_flag = True
			f = open("stats/lap_time_assignment.txt", 'a')
			f.write("End [s]: " + str(flag.end_lap) + "\n")
			f.close()
			if flag.dist_flag:
				fd = open("stats/distance_travelled_assignment.txt", 'a')
				fd.write("Tot distance (lap) [units]: " + str(flag.distance_travelled) + "\n\n")
				fd.close()
				flag.distance_travelled = 0
				flag.dist_flag = False
		
	if flag.start_flag and flag.end_flag:
		flag.start_flag = False
		flag.end_flag = False
		flag.lap_time = flag.end_lap - flag.start_lap
		f = open("stats/lap_time_assignment.txt", 'a')
		f.write("Lap [s]: " + str(flag.lap_time) + "\n\n")
		f.close()
