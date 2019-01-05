from pid import PID
import numpy as py
import math
import rospy

calss Steer_controler(object)
    def __init__(self, max_steer_angle)
	kp = 0.05
        ki = 0.0005
        kd = 0.5
        mn = -max_steer_angle
        mx = max_steer_angle

        self.steer_ctr = PID(kp, ki, kd, mn, mx)

    def control(self, pos, waypoints, sample_time)
	cte = self.cte_caculate(pos, waypoints)
	steer = self.steer_ctr.step(cte, sample_time)
	return steer

    def cte_caculate(self, pos, waypoints)
	#transfer waypoints into coordinate
	wp_carcoord_x, wp_carcoord_y = self.GloCoord_to_CarCoord(pos, waypoints)
	
	# Fitting third-order polynomial
	coeffs = np.polyfit(wp_carcoord_x[:10], wp_carcoord_y[:10], 3)
	poly = np.poly1d(coeffs)
	cte = poly(0.0)
	return cte

    def GloCoord_to_CarCoord(pos, waypoints)
	n = len(waypoints)

	#car's position
	car_x = pos.position.x
	car_y = pos.position.y

 	#car's orientation
	


