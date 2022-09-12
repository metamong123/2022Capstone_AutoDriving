import numpy as np

# paramters
dt = 0.1

# k =0.5
kp = 0.3 # control gain
# ki=0.01
# kd=0.5
ki = 0.01
kd = 0.7

# ERP42 PARAMETERS
LENGTH = 1.600
WIDTH = 1.160


# # GV70 PARAMETERS
# LENGTH = 4.715
# WIDTH = 1.910

def normalize_angle(angle):
	while angle > np.pi:
		angle -= 2.0 * np.pi

	while angle < -np.pi:
		angle += 2.0 * np.pi

	return angle


def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws, L, error_icte, prev_cte):
	# find nearest point
	global kp, ki, kd
	error_dcte=0
	min_dist = 1e9
	min_index = 0
	n_points = len(map_xs)

	back_x = x - L * np.cos(yaw)
	back_y = y - L * np.sin(yaw)

	for i in range(n_points):
		dx = - back_x + map_xs[i]
		dy = - back_y + map_ys[i]

		dist = np.sqrt(dx * dx + dy * dy)
		if dist < min_dist:
			min_dist = dist
			min_index = i

	# compute cte at front axle
	map_x = map_xs[min_index]
	map_y = map_ys[min_index]
	map_yaw = map_yaws[min_index]
	# dx = map_x - front_x
	# dy = map_y - front_y
	dx = back_x - map_x
	dy = back_y - map_y

	perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
	cte = np.dot([dx, dy], perp_vec)

	# control law
#	yaw_term = normalize_angle(map_yaw - yaw) * np.sin(np.pi/2 / (1+v/5))
	yaw_term = normalize_angle(map_yaw - yaw) #heading error
	error_dcte = cte - prev_cte
	error_icte += cte
	# cte_term = np.arctan2(kp*cte + ki*error_icte+ kd*error_dcte , (1.39+v)) # cross track error
	# cte_term = np.arctan2(kp*cte + ki*error_icte+ kd*error_dcte , (2.78+v)) # cross track error
	# cte_term = np.arctan2(kp*cte + ki*error_icte+ kd*error_dcte , (4.17+v)) # cross track error
	cte_term = np.arctan2((kp*cte + ki*error_icte+ kd*error_dcte) , (5.56+v)) # cross track error
	w_yaw = 0.9
	# w_cte = 0.65
	w_cte = 1
	# k =0.5
	# steering
	steer = w_yaw * yaw_term + w_cte * cte_term
	
	return steer, cte, [w_yaw, w_cte, kp, ki, kd, yaw_term, cte_term]
