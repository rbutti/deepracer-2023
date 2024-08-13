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
        racing_track = [[3.06804, 0.70038, 4.0, 0.03608],
                        [3.21384, 0.69221, 4.0, 0.03651],
                        [3.36119, 0.68719, 4.0, 0.03686],
                        [3.50966, 0.68459, 4.0, 0.03712],
                        [3.6589, 0.68362, 4.0, 0.03731],
                        [3.80856, 0.68352, 4.0, 0.03741],
                        [3.95832, 0.68357, 4.0, 0.03744],
                        [4.10808, 0.68362, 4.0, 0.03744],
                        [4.25783, 0.68367, 4.0, 0.03744],
                        [4.40759, 0.68373, 4.0, 0.03744],
                        [4.55735, 0.68378, 4.0, 0.03744],
                        [4.70711, 0.68384, 4.0, 0.03744],
                        [4.85687, 0.68389, 4.0, 0.03744],
                        [5.00663, 0.68395, 4.0, 0.03744],
                        [5.15639, 0.684, 3.70609, 0.04041],
                        [5.30615, 0.68405, 3.29864, 0.0454],
                        [5.45588, 0.68426, 2.99919, 0.04992],
                        [5.6052, 0.68644, 2.7574, 0.05416],
                        [5.7536, 0.69244, 2.57534, 0.05767],
                        [5.90038, 0.70405, 2.41915, 0.06087],
                        [6.04469, 0.72294, 2.2982, 0.06333],
                        [6.18548, 0.7506, 2.19319, 0.06542],
                        [6.32149, 0.78824, 2.12521, 0.0664],
                        [6.45124, 0.83684, 2.06687, 0.06703],
                        [6.57312, 0.8969, 2.02638, 0.06705],
                        [6.68534, 0.96857, 2.0096, 0.06626],
                        [6.78613, 1.05147, 2.0, 0.06525],
                        [6.87367, 1.14481, 2.0, 0.06398],
                        [6.94651, 1.24719, 2.0, 0.06282],
                        [7.00337, 1.35694, 2.0, 0.0618],
                        [7.04337, 1.47209, 2.0, 0.06095],
                        [7.06614, 1.59053, 2.0, 0.06031],
                        [7.07158, 1.71019, 2.00569, 0.05972],
                        [7.05995, 1.82908, 2.02711, 0.05893],
                        [7.03181, 1.94537, 2.06437, 0.05796],
                        [6.98798, 2.05743, 2.11173, 0.05698],
                        [6.92939, 2.16388, 2.16108, 0.05622],
                        [6.85694, 2.26344, 2.23215, 0.05516],
                        [6.77174, 2.35511, 2.32061, 0.05393],
                        [6.67499, 2.43814, 2.43735, 0.05231],
                        [6.56802, 2.5121, 2.55703, 0.05086],
                        [6.45205, 2.57673, 2.71093, 0.04898],
                        [6.32833, 2.63208, 2.89385, 0.04684],
                        [6.19809, 2.67846, 3.11762, 0.04434],
                        [6.06253, 2.71644, 3.40763, 0.04131],
                        [5.92275, 2.74688, 3.7849, 0.0378],
                        [5.77978, 2.77086, 4.0, 0.03624],
                        [5.6401, 2.79966, 4.0, 0.03566],
                        [5.50248, 2.83335, 4.0, 0.03542],
                        [5.36696, 2.87187, 4.0, 0.03522],
                        [5.23352, 2.91511, 4.0, 0.03507],
                        [5.10217, 2.963, 4.0, 0.03495],
                        [4.9729, 3.01541, 4.0, 0.03487],
                        [4.84566, 3.0722, 4.0, 0.03483],
                        [4.72039, 3.13316, 4.0, 0.03483],
                        [4.59702, 3.19809, 4.0, 0.03485],
                        [4.47544, 3.26673, 4.0, 0.0349],
                        [4.35553, 3.33878, 4.0, 0.03497],
                        [4.23715, 3.41392, 4.0, 0.03505],
                        [4.12016, 3.49185, 4.0, 0.03514],
                        [4.00444, 3.57231, 4.0, 0.03524],
                        [3.88988, 3.65505, 4.0, 0.03533],
                        [3.77636, 3.73981, 3.94144, 0.03594],
                        [3.66497, 3.82545, 3.68243, 0.03816],
                        [3.55238, 3.90875, 3.4626, 0.04045],
                        [3.43821, 3.98896, 3.27369, 0.04262],
                        [3.32211, 4.06536, 3.12327, 0.0445],
                        [3.20371, 4.13721, 3.00108, 0.04615],
                        [3.08266, 4.20376, 2.88427, 0.04789],
                        [2.95864, 4.26425, 2.77813, 0.04967],
                        [2.83133, 4.31785, 2.67962, 0.05155],
                        [2.70049, 4.3636, 2.59283, 0.05346],
                        [2.5661, 4.4005, 2.51589, 0.05539],
                        [2.42849, 4.42745, 2.45605, 0.05709],
                        [2.28855, 4.44321, 2.41804, 0.05824],
                        [2.14791, 4.44666, 2.39266, 0.0588],
                        [2.00886, 4.43703, 2.38499, 0.05844],
                        [1.87383, 4.41413, 2.38499, 0.05742],
                        [1.7449, 4.37834, 2.38499, 0.0561],
                        [1.62355, 4.33042, 2.38499, 0.05471],
                        [1.51062, 4.27133, 2.38499, 0.05344],
                        [1.40665, 4.20201, 2.38499, 0.0524],
                        [1.31188, 4.12334, 2.39658, 0.05139],
                        [1.22643, 4.03614, 2.42402, 0.05036],
                        [1.15033, 3.94118, 2.46306, 0.04941],
                        [1.08358, 3.83909, 2.52524, 0.0483],
                        [1.02611, 3.73052, 2.59971, 0.04725],
                        [0.97785, 3.61604, 2.67318, 0.04648],
                        [0.9388, 3.49612, 2.77587, 0.04544],
                        [0.90885, 3.37126, 2.89702, 0.04432],
                        [0.88783, 3.24196, 3.0294, 0.04324],
                        [0.87556, 3.10872, 3.15754, 0.04238],
                        [0.87192, 2.97194, 3.29351, 0.04154],
                        [0.87678, 2.83205, 3.438, 0.04071],
                        [0.88996, 2.68951, 3.41869, 0.04187],
                        [0.91129, 2.5448, 3.3039, 0.04427],
                        [0.94052, 2.39855, 3.18971, 0.04676],
                        [0.97639, 2.25557, 3.0979, 0.04758],
                        [1.01864, 2.11774, 3.02151, 0.04771],
                        [1.06724, 1.98537, 2.96629, 0.04754],
                        [1.12217, 1.85871, 2.9345, 0.04705],
                        [1.18339, 1.738, 2.92491, 0.04627],
                        [1.2509, 1.6235, 2.92491, 0.04545],
                        [1.32465, 1.5154, 2.92491, 0.04474],
                        [1.40459, 1.41392, 2.92491, 0.04417],
                        [1.49063, 1.3192, 2.92491, 0.04375],
                        [1.58264, 1.23137, 2.92491, 0.04349],
                        [1.68045, 1.15051, 2.93697, 0.04321],
                        [1.78388, 1.07666, 2.97142, 0.04277],
                        [1.8927, 1.00983, 3.02938, 0.04216],
                        [2.00669, 0.94996, 3.11268, 0.04136],
                        [2.12556, 0.89697, 3.20597, 0.04059],
                        [2.24905, 0.85074, 3.32575, 0.03965],
                        [2.37687, 0.81114, 3.485, 0.0384],
                        [2.50869, 0.77789, 3.68113, 0.03693],
                        [2.64411, 0.75066, 3.91727, 0.03526],
                        [2.78276, 0.72901, 4.0, 0.03508],
                        [2.92421, 0.71245, 4.0, 0.0356]]

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