import numpy as np

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


	def find_nearest_point(self, front_x, front_y, map_xs, map_ys):
		min_dist = 1e3
		n_points = len(map_xs)

		for i in range(n_points):
			dx = front_x - map_xs[i]
			dy = front_y - map_ys[i]

			dist = np.sqrt(dx * dx + dy * dy)
			if dist < min_dist:
				min_dist = dist
				min_index = i
		
		return min_index

    
	def stanley_control(self, x, y, yaw, v, map_xs, map_ys, map_yaws):
		front_x = x + self.WB/2*np.cos(yaw)
		front_y = y + self.WB/2*np.sin(yaw)

		# find nearest point
		min_index = self.find_nearest_point(front_x, front_y, map_xs, map_ys)
        
		map_x = map_xs[min_index]
		map_y = map_ys[min_index]
		map_yaw = map_yaws[min_index]
		dx = map_x - front_x
		dy = map_y - front_y

		# compute cte at front axle
		perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
		cte = np.dot([dx, dy], perp_vec)

		# control law
		yaw_term = self.normalize_angle(map_yaw - yaw) # heading error
		cte_term = np.arctan2(self.k*cte , (self.speed_gain + v)) # cross track error

		# steering
		steer = self.w_yaw * yaw_term + self.w_cte * cte_term
		return steer, yaw_term, cte


	def stanley_control_pd(self, x, y, yaw, v, map_xs, map_ys, map_yaws):
		front_x = x + self.WB/2*np.cos(yaw)
		front_y = y + self.WB/2*np.sin(yaw)

		# find nearest point
		min_index = self.find_nearest_point(front_x, front_y, map_xs, map_ys)

		map_x = map_xs[min_index]
		map_y = map_ys[min_index]
		map_yaw = map_yaws[min_index]
		dx = map_x - front_x
		dy = map_y - front_y

		# compute cte at front axle
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