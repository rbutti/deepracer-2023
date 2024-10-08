import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
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
                        [2.11566, -4.51158, 3.64727, 0.0826],
                        [2.37975, -4.65651, 3.17171, 0.09498],
                        [2.6438, -4.80153, 2.84231, 0.10599],
                        [2.9078, -4.94666, 2.58793, 0.11641],
                        [3.17182, -5.09174, 2.38841, 0.12613],
                        [3.43588, -5.23646, 2.22539, 0.13531],
                        [3.7002, -5.37841, 2.08084, 0.14418],
                        [3.96501, -5.51528, 1.95481, 0.15249],
                        [4.23044, -5.6448, 1.84394, 0.16017],
                        [4.49651, -5.76483, 1.74344, 0.16743],
                        [4.76314, -5.8734, 1.64888, 0.17459],
                        [5.0301, -5.96861, 1.56332, 0.18131],
                        [5.29712, -6.04877, 1.56332, 0.17833],
                        [5.56376, -6.1123, 1.56332, 0.17534],
                        [5.8295, -6.15764, 1.56332, 0.17244],
                        [6.09368, -6.18323, 1.56332, 0.16977],
                        [6.35545, -6.18753, 1.56332, 0.16747],
                        [6.61376, -6.16884, 1.56332, 0.16567],
                        [6.86721, -6.12523, 1.56332, 0.16451],
                        [7.11387, -6.05455, 1.64613, 0.15587],
                        [7.35304, -5.9598, 1.67385, 0.15369],
                        [7.58369, -5.84191, 1.69561, 0.15277],
                        [7.80467, -5.70146, 1.71484, 0.15269],
                        [8.01466, -5.53885, 1.72158, 0.15427],
                        [8.21207, -5.35435, 1.7094, 0.15807],
                        [8.39498, -5.14835, 1.69874, 0.16217],
                        [8.56097, -4.92144, 1.68327, 0.16702],
                        [8.70707, -4.675, 1.66361, 0.17221],
                        [8.8299, -4.41182, 1.643, 0.17677],
                        [8.9264, -4.13646, 1.61175, 0.18104],
                        [8.99436, -3.85443, 1.57727, 0.18393],
                        [9.0331, -3.57112, 1.54121, 0.18553],
                        [9.04311, -3.29087, 1.5, 0.18695],
                        [9.02561, -3.01689, 1.5, 0.18303],
                        [8.98196, -2.7515, 1.5, 0.1793],
                        [8.91343, -2.49652, 1.5, 0.17602],
                        [8.82111, -2.25347, 1.5, 0.17333],
                        [8.70568, -2.02391, 1.5, 0.1713],
                        [8.56755, -1.80952, 1.5, 0.17002],
                        [8.40698, -1.61228, 1.5, 0.16956],
                        [8.22398, -1.43468, 1.58124, 0.16127],
                        [8.02177, -1.27626, 1.67057, 0.15377],
                        [7.80295, -1.13637, 1.76559, 0.14709],
                        [7.56967, -1.01431, 1.87524, 0.1404],
                        [7.32383, -0.9092, 2.01114, 0.13295],
                        [7.06723, -0.81985, 2.16909, 0.12526],
                        [6.80151, -0.7449, 2.36925, 0.11653],
                        [6.52823, -0.68274, 2.63283, 0.10645],
                        [6.24891, -0.63145, 3.02538, 0.09387],
                        [5.96504, -0.58875, 3.1978, 0.08977],
                        [5.67813, -0.55202, 2.96404, 0.09759],
                        [5.38964, -0.51849, 2.78924, 0.10413],
                        [5.09657, -0.4841, 2.65441, 0.11117],
                        [4.80394, -0.44772, 2.54979, 0.11565],
                        [4.51216, -0.4076, 2.4718, 0.11915],
                        [4.22166, -0.3621, 2.41767, 0.12162],
                        [3.93292, -0.30961, 2.38304, 0.12315],
                        [3.64646, -0.24871, 2.36105, 0.12404],
                        [3.36288, -0.17815, 2.35571, 0.12405],
                        [3.08284, -0.0968, 2.35571, 0.12379],
                        [2.80709, -0.00375, 2.35571, 0.12354],
                        [2.53647, 0.10173, 2.35571, 0.1233],
                        [2.27185, 0.22016, 2.35571, 0.12307],
                        [2.01411, 0.35174, 2.35571, 0.12285],
                        [1.76407, 0.49647, 2.35571, 0.12264],
                        [1.52249, 0.65413, 2.35571, 0.12246],
                        [1.29001, 0.82433, 2.36659, 0.12174],
                        [1.06711, 1.00648, 2.39458, 0.12021],
                        [0.85407, 1.19986, 2.43468, 0.11818],
                        [0.65099, 1.40368, 2.48583, 0.11574],
                        [0.45786, 1.61714, 2.55857, 0.11251],
                        [0.27445, 1.8394, 2.62608, 0.10973],
                        [0.10056, 2.06972, 2.70772, 0.10658],
                        [-0.06412, 2.30739, 2.80919, 0.10293],
                        [-0.22002, 2.55168, 2.89345, 0.10016],
                        [-0.36766, 2.80189, 2.60253, 0.11163],
                        [-0.50761, 3.05734, 2.37989, 0.12239],
                        [-0.64045, 3.3173, 2.19811, 0.13282],
                        [-0.76654, 3.58034, 2.05175, 0.14217],
                        [-0.88374, 3.83934, 1.91668, 0.14832],
                        [-1.00418, 4.09277, 1.79824, 0.15604],
                        [-1.13001, 4.34044, 1.79824, 0.15449],
                        [-1.26272, 4.58096, 1.79824, 0.15276],
                        [-1.40369, 4.81302, 1.79824, 0.15099],
                        [-1.55423, 5.03537, 1.79824, 0.14932],
                        [-1.71562, 5.24662, 1.79824, 0.14784],
                        [-1.88899, 5.44533, 1.79824, 0.14665],
                        [-2.07556, 5.62966, 1.79824, 0.14585],
                        [-2.27651, 5.79739, 1.8956, 0.13808],
                        [-2.4899, 5.94978, 1.94792, 0.13462],
                        [-2.71476, 6.08718, 1.99647, 0.13199],
                        [-2.9503, 6.20977, 2.0424, 0.13001],
                        [-3.19587, 6.31759, 2.08936, 0.12837],
                        [-3.45089, 6.41063, 2.13346, 0.12724],
                        [-3.71483, 6.48872, 2.17911, 0.12631],
                        [-3.98714, 6.55171, 2.20212, 0.12692],
                        [-4.26729, 6.59927, 2.16844, 0.13105],
                        [-4.55469, 6.63103, 2.13144, 0.13566],
                        [-4.84852, 6.64657, 2.09407, 0.14051],
                        [-5.14742, 6.64497, 2.0532, 0.14558],
                        [-5.44424, 6.62585, 2.01396, 0.14769],
                        [-5.73445, 6.58967, 1.9719, 0.14831],
                        [-6.01724, 6.53702, 1.93762, 0.14846],
                        [-6.29216, 6.4683, 1.9048, 0.14877],
                        [-6.55883, 6.3838, 1.86832, 0.14972],
                        [-6.8168, 6.28368, 1.84242, 0.15019],
                        [-7.06561, 6.16801, 1.81486, 0.15118],
                        [-7.30464, 6.03672, 1.79145, 0.15223],
                        [-7.53318, 5.88968, 1.76126, 0.1543],
                        [-7.75032, 5.72668, 1.73248, 0.15672],
                        [-7.95501, 5.5476, 1.71418, 0.15866],
                        [-8.14595, 5.35238, 1.7078, 0.15989],
                        [-8.32146, 5.1411, 1.70672, 0.16093],
                        [-8.47968, 4.91435, 1.70256, 0.1624],
                        [-8.61841, 4.67324, 1.69629, 0.16399],
                        [-8.73538, 4.41973, 1.68188, 0.166],
                        [-8.82826, 4.15654, 1.66584, 0.16755],
                        [-8.89516, 3.88706, 1.63207, 0.17013],
                        [-8.93517, 3.61498, 1.63207, 0.1685],
                        [-8.94851, 3.34361, 1.63207, 0.16647],
                        [-8.93596, 3.0756, 1.63207, 0.16439],
                        [-8.89842, 2.81298, 1.63207, 0.16255],
                        [-8.83669, 2.55733, 1.63207, 0.16114],
                        [-8.75122, 2.31009, 1.63207, 0.16029],
                        [-8.64221, 2.07259, 1.63207, 0.16012],
                        [-8.50914, 1.84649, 1.77566, 0.14775],
                        [-8.35585, 1.63103, 1.87023, 0.14139],
                        [-8.18422, 1.42601, 1.98048, 0.135],
                        [-7.99599, 1.2311, 2.10821, 0.12853],
                        [-7.7928, 1.04583, 2.25665, 0.12185],
                        [-7.57625, 0.86962, 2.43779, 0.11452],
                        [-7.34797, 0.70173, 2.6579, 0.10661],
                        [-7.10957, 0.5413, 2.93496, 0.09791],
                        [-6.86265, 0.3873, 3.30307, 0.0881],
                        [-6.60878, 0.23859, 3.83442, 0.07673],
                        [-6.34948, 0.09384, 4.0, 0.07424],
                        [-6.08616, -0.0484, 4.0, 0.07482],
                        [-5.82027, -0.18966, 4.0, 0.07527],
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