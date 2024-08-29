import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

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
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

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
            if start is None:
                start = 0
            if end is None:
                end = 0
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[3.06217, 0.68808, 3.77403, 0.03894],
                        [3.21032, 0.68455, 4.0, 0.03705],
                        [3.35941, 0.68324, 4.0, 0.03727],
                        [3.50903, 0.68319, 4.0, 0.0374],
                        [3.65879, 0.68342, 4.0, 0.03744],
                        [3.80856, 0.68352, 4.0, 0.03744],
                        [3.95832, 0.68357, 4.0, 0.03744],
                        [4.10808, 0.68362, 4.0, 0.03744],
                        [4.25783, 0.68367, 4.0, 0.03744],
                        [4.40759, 0.68373, 4.0, 0.03744],
                        [4.55735, 0.68378, 4.0, 0.03744],
                        [4.70711, 0.68384, 3.41318, 0.04388],
                        [4.85687, 0.68389, 2.83519, 0.05282],
                        [5.00663, 0.68395, 2.47438, 0.06052],
                        [5.15639, 0.684, 2.21947, 0.06748],
                        [5.30615, 0.68405, 2.02195, 0.07407],
                        [5.45583, 0.68451, 1.87392, 0.07987],
                        [5.60502, 0.68718, 1.75386, 0.08508],
                        [5.7532, 0.69379, 1.64, 0.09045],
                        [5.89972, 0.70599, 1.55102, 0.09479],
                        [6.04377, 0.72526, 1.47185, 0.09874],
                        [6.18439, 0.75295, 1.41393, 0.10137],
                        [6.32046, 0.79019, 1.37196, 0.10282],
                        [6.45071, 0.83782, 1.33808, 0.10365],
                        [6.57377, 0.89638, 1.3114, 0.10392],
                        [6.68794, 0.96627, 1.3, 0.10298],
                        [6.79149, 1.04745, 1.3, 0.10121],
                        [6.8825, 1.13948, 1.3, 0.09956],
                        [6.95918, 1.24137, 1.3, 0.09809],
                        [7.01998, 1.35156, 1.3, 0.09682],
                        [7.06356, 1.46817, 1.3, 0.09575],
                        [7.08889, 1.58892, 1.3, 0.09491],
                        [7.09554, 1.71135, 1.30916, 0.09365],
                        [7.08391, 1.83301, 1.32254, 0.09241],
                        [7.05464, 1.95172, 1.34226, 0.09109],
                        [7.00862, 2.06554, 1.3789, 0.08904],
                        [6.94713, 2.17293, 1.41895, 0.08721],
                        [6.87145, 2.2726, 1.47916, 0.08461],
                        [6.78307, 2.36372, 1.54797, 0.082],
                        [6.68347, 2.44574, 1.63003, 0.07916],
                        [6.57405, 2.5184, 1.72265, 0.07625],
                        [6.45616, 2.5817, 1.83503, 0.07292],
                        [6.33106, 2.63588, 1.94888, 0.06995],
                        [6.19985, 2.68125, 2.09809, 0.06617],
                        [6.06362, 2.71841, 2.25789, 0.06254],
                        [5.9233, 2.74801, 2.45864, 0.05833],
                        [5.77978, 2.77086, 2.68316, 0.05416],
                        [5.63989, 2.79918, 2.73323, 0.05222],
                        [5.50224, 2.83281, 2.77903, 0.05099],
                        [5.36677, 2.87145, 2.80664, 0.0502],
                        [5.23343, 2.91494, 2.84873, 0.04923],
                        [5.10219, 2.96305, 2.88204, 0.0485],
                        [4.973, 3.01563, 2.93023, 0.0476],
                        [4.8458, 3.07251, 2.98027, 0.04675],
                        [4.72056, 3.13353, 3.06529, 0.04545],
                        [4.59719, 3.19846, 3.1715, 0.04396],
                        [4.47558, 3.26703, 3.3104, 0.04217],
                        [4.35562, 3.33896, 3.44682, 0.04058],
                        [4.23719, 3.41401, 3.47168, 0.04038],
                        [4.12017, 3.49188, 3.09542, 0.04541],
                        [4.00444, 3.57232, 2.82209, 0.04994],
                        [3.88988, 3.65504, 2.60496, 0.05425],
                        [3.77636, 3.73981, 2.42963, 0.05831],
                        [3.66497, 3.82546, 2.28489, 0.0615],
                        [3.55239, 3.90877, 2.1582, 0.0649],
                        [3.43824, 3.98902, 2.05238, 0.06799],
                        [3.32216, 4.06547, 1.95574, 0.07107],
                        [3.20379, 4.13738, 1.87688, 0.07379],
                        [3.08278, 4.204, 1.81602, 0.07607],
                        [2.95878, 4.26455, 1.77405, 0.07779],
                        [2.83148, 4.31821, 1.7375, 0.07951],
                        [2.70065, 4.36401, 1.71748, 0.08071],
                        [2.56625, 4.40087, 1.69636, 0.08216],
                        [2.4286, 4.4275, 1.67131, 0.08389],
                        [2.28873, 4.44262, 1.64411, 0.08557],
                        [2.14839, 4.44522, 1.6174, 0.08678],
                        [2.00982, 4.43488, 1.57827, 0.08804],
                        [1.87516, 4.41176, 1.53702, 0.08889],
                        [1.74606, 4.37659, 1.4961, 0.08944],
                        [1.62365, 4.33019, 1.4961, 0.0875],
                        [1.50871, 4.27333, 1.4961, 0.08571],
                        [1.40186, 4.20666, 1.4961, 0.08418],
                        [1.30361, 4.13072, 1.4961, 0.083],
                        [1.21462, 4.04583, 1.4961, 0.08221],
                        [1.13571, 3.95217, 1.4961, 0.08186],
                        [1.0679, 3.84989, 1.53793, 0.07979],
                        [1.01118, 3.73996, 1.67803, 0.07372],
                        [0.9643, 3.6239, 1.75039, 0.07151],
                        [0.92703, 3.50238, 1.82879, 0.0695],
                        [0.89912, 3.376, 1.91547, 0.06757],
                        [0.88033, 3.2453, 2.01207, 0.06562],
                        [0.87039, 3.1108, 2.11571, 0.06374],
                        [0.86902, 2.97299, 2.23628, 0.06163],
                        [0.87582, 2.83237, 2.37056, 0.05939],
                        [0.89035, 2.68943, 2.34161, 0.06136],
                        [0.91211, 2.54465, 2.2149, 0.0661],
                        [0.94052, 2.39855, 2.10322, 0.07077],
                        [0.97439, 2.254, 2.00236, 0.07414],
                        [1.01306, 2.11309, 1.91277, 0.07639],
                        [1.05694, 1.97634, 1.81981, 0.07892],
                        [1.10644, 1.84427, 1.73298, 0.08139],
                        [1.16188, 1.71738, 1.6549, 0.08368],
                        [1.22355, 1.59613, 1.6549, 0.0822],
                        [1.29168, 1.48099, 1.6549, 0.08084],
                        [1.36652, 1.37247, 1.6549, 0.07966],
                        [1.44828, 1.27108, 1.6549, 0.0787],
                        [1.53729, 1.17755, 1.6549, 0.07802],
                        [1.63392, 1.09275, 1.6549, 0.07769],
                        [1.73855, 1.01773, 1.74949, 0.07359],
                        [1.85012, 0.95182, 1.83838, 0.07049],
                        [1.96786, 0.89453, 1.94076, 0.06747],
                        [2.09108, 0.84536, 2.04844, 0.06476],
                        [2.21921, 0.8039, 2.16901, 0.06209],
                        [2.35172, 0.7697, 2.30907, 0.05927],
                        [2.4881, 0.74228, 2.47502, 0.0562],
                        [2.62784, 0.72109, 2.67763, 0.05278],
                        [2.77043, 0.7055, 2.93769, 0.04883],
                        [2.91538, 0.69477, 3.28387, 0.04426]]

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
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 2  # Increased multiple for more strict line following
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)) ** 2)  # Quadratic punishment for deviation
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 0.5
        SPEED_MULTIPLE = 3  # Increased speed multiple to encourage faster driving
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 11
        FASTEST_TIME = 8
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                            steps_prediction - (STANDARD_TIME * 15 + 1)))
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
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        # Constants
        MAX_REWARD = 1000  # Maximum reward for completing the track
        REWARD_FOR_FASTEST_TIME = 1000  # Increase to make completing the lap faster more desirable
        STANDARD_TIME = 37  # seconds (time that is easily done by the model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)

        # Calculate reward based on progress
        progress_reward = (progress / 100) * MAX_REWARD

        # Calculate finish reward based on fastest time
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        # Combine progress reward and finish reward
        reward = progress_reward + finish_reward
        # Ensure a minimum reward to avoid zero reward cases
        reward = max(reward, 1e-3)
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


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)