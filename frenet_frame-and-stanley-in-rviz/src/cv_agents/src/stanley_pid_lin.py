import numpy as np
from math import sqrt
from scipy import interpolate

class Stanley:
	def __init__(self, k, speed_gain, w_yaw, w_cte,  cte_thresh = 0.5, yaw_dgain = 0, WB = 1.04):
		self.WB = WB
        
		self.k = k
		self.speed_gain = speed_gain
		self.w_yaw = w_yaw
		self.w_cte = w_cte

		# parameter for stanley_control_thresh
		self.cte_thresh = cte_thresh
	
		# parameter for stanley_control_pd
		self.yaw_dgain = yaw_dgain
		self.prev_yaw = 0


	def normalize_angle(self, angle):
		while angle > np.pi:
			angle -= 2.0 * np.pi

		while angle < -np.pi:
			angle += 2.0 * np.pi

		return angle


	def get_map_state(self, front_x, front_y, map_xs, map_ys, map_yaws):
		min_dist = 1e3
		cte_index = 0
		yaw_index = 0

		if len(map_xs) == 1:
			cte_index = 0
			map_yaw = map_yaws[0]
		else:
			# get_closest waypoint for cte
			for i in range(len(map_xs)):
				dx = front_x - map_xs[i]
				dy = front_y - map_ys[i]

				dist = np.sqrt(dx * dx + dy * dy)
				if dist < min_dist:
					min_dist = dist
					cte_index = i

			# get yaw_index for yaw_term
			map_vec = np.array([map_xs[cte_index + 1] - map_xs[cte_index], map_ys[cte_index + 1] - map_ys[cte_index]])
			ego_vec = np.array([front_x - map_xs[cte_index], front_y - map_ys[cte_index]])
			direction  = np.sign(np.dot(map_vec, ego_vec))

			if direction < 0:
				yaw_index = cte_index - 1
			else:
				yaw_index = cte_index

			# get map_yaw: linear interpolation
			map_dist = sqrt((map_xs[yaw_index + 1] - map_xs[yaw_index])**2 + (map_ys[yaw_index + 1] - map_ys[yaw_index])**2)
			map_car_dist = abs(np.dot(map_vec/map_dist, ego_vec))

			yaw_linear = interpolate.interp1d([0, map_dist], [map_yaws[yaw_index], map_yaws[yaw_index+1]], kind='linear')
			map_yaw = yaw_linear(map_car_dist)

		return map_xs[cte_index], map_ys[cte_index], map_yaw


	def stanley_control_pd(self, x, y, yaw, v, map_xs, map_ys, map_yaws):
		front_x = x + self.WB/2*np.cos(yaw)
		front_y = y + self.WB/2*np.sin(yaw)

		# get map state of closest waypoint
		map_x, map_y, map_yaw = self.get_map_state(front_x, front_y, map_xs, map_ys, map_yaws)

		# compute cte at front axle
		dx = map_x - front_x
		dy = map_y - front_y
		perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
		cte = np.dot([dx, dy], perp_vec)

		# cte threshold
		#if -self.cte_thresh < cte < self.cte_thresh: # is this okay?
		#	cte = cte**3

		# heading error
		yaw_term = self.normalize_angle(map_yaw - yaw)
		d_yaw = yaw_term - self.prev_yaw
		self.prev_yaw = yaw_term

		# cross track error (cte)
		cte_term = np.arctan2(self.k*cte, (self.speed_gain + v))

		# steering
		steer = self.w_yaw*yaw_term + self.yaw_dgain*d_yaw + self.w_cte*cte_term
		return steer, yaw_term, cte, map_yaw
