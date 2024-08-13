import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = -1
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.11087, -3.28924, 4.0, 0.04281],
                        [0.00309, -3.3518, 4.0, 0.0325],
                        [0.11704, -3.41436, 4.0, 0.0325],
                        [0.26716, -3.49678, 4.0, 0.04281],
                        [0.53122, -3.64176, 4.0, 0.07531],
                        [0.79529, -3.78674, 4.0, 0.07531],
                        [1.05936, -3.93172, 4.0, 0.07531],
                        [1.32342, -4.07671, 4.0, 0.07531],
                        [1.58749, -4.22169, 4.0, 0.07531],
                        [1.85157, -4.36664, 4.0, 0.07531],
                        [2.11566, -4.51158, 4.0, 0.07531],
                        [2.37975, -4.65651, 3.86321, 0.07798],
                        [2.6438, -4.80153, 3.4466, 0.08741],
                        [2.9078, -4.94666, 3.14255, 0.09586],
                        [3.17182, -5.09174, 2.89927, 0.10391],
                        [3.43585, -5.2368, 2.70176, 0.1115],
                        [3.70012, -5.37916, 2.53634, 0.11835],
                        [3.96493, -5.51615, 2.41296, 0.12356],
                        [4.2304, -5.64518, 2.32315, 0.12706],
                        [4.49652, -5.76382, 2.25805, 0.12903],
                        [4.76312, -5.8698, 2.2124, 0.12967],
                        [5.02989, -5.96114, 2.18271, 0.12919],
                        [5.29641, -6.03603, 2.16536, 0.12785],
                        [5.56213, -6.09284, 2.15828, 0.1259],
                        [5.82635, -6.13012, 2.15828, 0.12364],
                        [6.08832, -6.14681, 2.15828, 0.12162],
                        [6.3472, -6.14222, 2.15828, 0.11996],
                        [6.60209, -6.11592, 2.15828, 0.11873],
                        [6.85205, -6.06767, 2.15828, 0.11795],
                        [7.0961, -5.99735, 2.15828, 0.11767],
                        [7.33315, -5.9049, 2.15828, 0.11789],
                        [7.56204, -5.79031, 2.16161, 0.11842],
                        [7.78151, -5.65368, 2.17176, 0.11904],
                        [7.99011, -5.49517, 2.18611, 0.11985],
                        [8.18622, -5.31508, 2.16483, 0.12299],
                        [8.3679, -5.11388, 2.12823, 0.12738],
                        [8.53291, -4.89246, 2.09066, 0.13208],
                        [8.67873, -4.65241, 2.05571, 0.13663],
                        [8.80266, -4.39623, 2.02619, 0.14045],
                        [8.90206, -4.12745, 2.00613, 0.14285],
                        [8.97471, -3.85048, 2.0, 0.14317],
                        [9.0191, -3.5701, 2.0, 0.14194],
                        [9.0347, -3.29087, 2.0, 0.13983],
                        [9.02173, -3.01672, 2.0, 0.13723],
                        [8.98101, -2.7508, 2.0, 0.13451],
                        [8.91362, -2.49558, 2.0, 0.13198],
                        [8.82077, -2.25303, 2.0, 0.12986],
                        [8.70375, -2.02472, 2.0, 0.12828],
                        [8.56395, -1.81191, 2.00973, 0.1267],
                        [8.40281, -1.61559, 2.03613, 0.12474],
                        [8.22184, -1.43649, 2.08012, 0.1224],
                        [8.02259, -1.27512, 2.14318, 0.11963],
                        [7.80665, -1.13172, 2.22762, 0.11637],
                        [7.57566, -1.00624, 2.33693, 0.11249],
                        [7.33128, -0.8983, 2.47656, 0.10787],
                        [7.07522, -0.80717, 2.65568, 0.10234],
                        [6.80923, -0.7317, 2.89061, 0.09565],
                        [6.53503, -0.67029, 3.21259, 0.08747],
                        [6.25433, -0.62091, 3.6874, 0.07729],
                        [5.96882, -0.58107, 3.96006, 0.0728],
                        [5.68008, -0.54795, 3.67613, 0.07906],
                        [5.38964, -0.51849, 3.48837, 0.08368],
                        [5.09574, -0.48814, 3.33343, 0.08864],
                        [4.80232, -0.45535, 3.21142, 0.09194],
                        [4.50979, -0.41831, 3.11577, 0.09464],
                        [4.21857, -0.37527, 3.04302, 0.09674],
                        [3.92913, -0.32465, 2.99262, 0.09819],
                        [3.64197, -0.26506, 2.96321, 0.09897],
                        [3.35772, -0.19516, 2.95323, 0.09912],
                        [3.07705, -0.11396, 2.95323, 0.09894],
                        [2.80072, -0.0206, 2.95323, 0.09876],
                        [2.52963, 0.08561, 2.95323, 0.09859],
                        [2.26472, 0.20513, 2.95323, 0.09841],
                        [2.00698, 0.33822, 2.95323, 0.09822],
                        [1.75735, 0.48486, 2.95323, 0.09803],
                        [1.51671, 0.6448, 2.95323, 0.09784],
                        [1.28576, 0.81758, 2.96355, 0.09733],
                        [1.06502, 1.00259, 2.99716, 0.09609],
                        [0.8548, 1.19904, 3.05929, 0.09405],
                        [0.65508, 1.40603, 3.16659, 0.09083],
                        [0.46553, 1.62249, 3.28748, 0.08752],
                        [0.28571, 1.84745, 3.45582, 0.08334],
                        [0.11491, 2.07987, 3.6525, 0.07897],
                        [-0.04761, 2.31881, 3.90917, 0.07392],
                        [-0.20277, 2.56333, 3.80639, 0.07608],
                        [-0.35141, 2.81259, 3.45713, 0.08395],
                        [-0.49448, 3.06579, 3.20886, 0.09063],
                        [-0.63296, 3.32215, 3.02651, 0.09627],
                        [-0.76776, 3.58098, 2.8914, 0.10093],
                        [-0.89789, 3.82433, 2.79414, 0.09876],
                        [-1.03152, 4.06397, 2.7269, 0.10062],
                        [-1.17, 4.29847, 2.68415, 0.10146],
                        [-1.31453, 4.52649, 2.6622, 0.10141],
                        [-1.46617, 4.74683, 2.63951, 0.10133],
                        [-1.6258, 4.95842, 2.63408, 0.10062],
                        [-1.79411, 5.16035, 2.63408, 0.0998],
                        [-1.97163, 5.35183, 2.63408, 0.09913],
                        [-2.15874, 5.53218, 2.63408, 0.09866],
                        [-2.35564, 5.70082, 2.63408, 0.09842],
                        [-2.56244, 5.85726, 2.63408, 0.09844],
                        [-2.77912, 6.00108, 2.63408, 0.09873],
                        [-3.00571, 6.1317, 2.63408, 0.09929],
                        [-3.24215, 6.2486, 2.65173, 0.09947],
                        [-3.48821, 6.35138, 2.66014, 0.10025],
                        [-3.74375, 6.43942, 2.6779, 0.10093],
                        [-4.00853, 6.51208, 2.7051, 0.1015],
                        [-4.28217, 6.56872, 2.7265, 0.10249],
                        [-4.5641, 6.6086, 2.69112, 0.1058],
                        [-4.85329, 6.63095, 2.6522, 0.10936],
                        [-5.14763, 6.63491, 2.60913, 0.11282],
                        [-5.44271, 6.62006, 2.56289, 0.11528],
                        [-5.73332, 6.58682, 2.51651, 0.11623],
                        [-6.01678, 6.53601, 2.47617, 0.1163],
                        [-6.29223, 6.46842, 2.42963, 0.11673],
                        [-6.55921, 6.38452, 2.38536, 0.11732],
                        [-6.81735, 6.28464, 2.34442, 0.11806],
                        [-7.06618, 6.16897, 2.31587, 0.11849],
                        [-7.30516, 6.03754, 2.29142, 0.11902],
                        [-7.53358, 5.89029, 2.26551, 0.11996],
                        [-7.7506, 5.72707, 2.23592, 0.12144],
                        [-7.95518, 5.54783, 2.20295, 0.12347],
                        [-8.14599, 5.35245, 2.16912, 0.1259],
                        [-8.32137, 5.14104, 2.1392, 0.1284],
                        [-8.47933, 4.91411, 2.1187, 0.1305],
                        [-8.61776, 4.67289, 2.11226, 0.13167],
                        [-8.73453, 4.41939, 2.11226, 0.13213],
                        [-8.8276, 4.15634, 2.11226, 0.1321],
                        [-8.89531, 3.88701, 2.11226, 0.13148],
                        [-8.93654, 3.61488, 2.11226, 0.13031],
                        [-8.95079, 3.34327, 2.11226, 0.12876],
                        [-8.93817, 3.07511, 2.11226, 0.1271],
                        [-8.89928, 2.81275, 2.11226, 0.12557],
                        [-8.83509, 2.55795, 2.12303, 0.12376],
                        [-8.74678, 2.31199, 2.15297, 0.12138],
                        [-8.63565, 2.07572, 2.2039, 0.11847],
                        [-8.5031, 1.84965, 2.27851, 0.11502],
                        [-8.35057, 1.63402, 2.381, 0.11093],
                        [-8.17962, 1.4288, 2.51581, 0.10617],
                        [-7.99192, 1.23372, 2.68242, 0.10092],
                        [-7.78918, 1.04828, 2.87841, 0.09545],
                        [-7.57308, 0.87186, 3.11049, 0.08969],
                        [-7.34523, 0.70374, 3.3922, 0.08347],
                        [-7.10727, 0.54303, 3.75265, 0.07652],
                        [-6.8608, 0.38871, 4.0, 0.0727],
                        [-6.60735, 0.23963, 4.0, 0.07351],
                        [-6.34844, 0.09449, 4.0, 0.0742],
                        [-6.08554, -0.04813, 4.0, 0.07478],
                        [-5.82027, -0.18966, 4.0, 0.07516],
                        [-5.55444, -0.33114, 4.0, 0.07529],
                        [-5.28872, -0.47287, 4.0, 0.07529],
                        [-5.02313, -0.61483, 4.0, 0.07529],
                        [-4.75766, -0.75703, 4.0, 0.07529],
                        [-4.49232, -0.89949, 4.0, 0.07529],
                        [-4.2271, -1.0422, 4.0, 0.07529],
                        [-3.96201, -1.18514, 4.0, 0.0753],
                        [-3.69703, -1.32832, 4.0, 0.0753],
                        [-3.43217, -1.47174, 4.0, 0.0753],
                        [-3.16742, -1.61538, 4.0, 0.0753],
                        [-2.9028, -1.75926, 4.0, 0.0753],
                        [-2.63829, -1.90337, 4.0, 0.0753],
                        [-2.3739, -2.04771, 4.0, 0.07531],
                        [-2.10959, -2.19221, 4.0, 0.07531],
                        [-1.84539, -2.33694, 4.0, 0.07531],
                        [-1.58133, -2.48192, 4.0, 0.07531],
                        [-1.31726, -2.6269, 4.0, 0.07531],
                        [-1.05319, -2.77189, 4.0, 0.07531],
                        [-0.78912, -2.91687, 4.0, 0.07531],
                        [-0.52505, -3.06184, 4.0, 0.07531],
                        [-0.26098, -3.20682, 4.0, 0.07531]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)