import numpy as np

class Stanley:
    def __init__(self, k, speed_gain, w_yaw, w_cte,  cte_thresh = 0.5, p_gain = 1, i_gain = 1, d_gain = 1, WB = 1.04):
        self.WB = WB
        
        self.k = k
        self.speed_gain = speed_gain
        self.w_yaw = w_yaw
        self.w_cte = w_cte

        # parameter for 
        self.cte_thresh = cte_thresh

        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

        self.error_icte = 0
        self.prev_cte = 0


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
        return steer


    def stanley_control_thresh(self, x, y, yaw, v, map_xs, map_ys, map_yaws):
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

        if -self.cte_thresh < cte < self.cte_thresh: # 이게 맞나?
            cte = cte**3

        # control law
        yaw_term = self.normalize_angle(map_yaw - yaw) # heading error
        cte_term = np.arctan2(self.k*cte , (self.speed_gain + v)) # cross track error

        # steering
        steer = self.w_yaw * yaw_term + self.w_cte * cte_term
        return steer


    def stanley_control_pid(self, x, y, yaw, v, map_xs, map_ys, map_yaws):
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
        error_dcte = cte - self.prev_cte
        self.prev_cte = cte
        self.error_icte += cte
        cte_term = np.arctan2(self.p_gain*cte + self.i_gain*self.error_icte + self.d_gain*error_dcte, (self.speed_gain + v)) # cross track error

        # steering
        steer = self.w_yaw * yaw_term + self.w_cte * cte_term
        return steer
