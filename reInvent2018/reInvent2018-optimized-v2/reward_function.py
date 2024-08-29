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
        racing_track = [[3.06217, 0.68808, 4.0, 0.03674],
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
                        [4.70711, 0.68384, 4.0, 0.03744],
                        [4.85687, 0.68389, 3.67573, 0.04074],
                        [5.00663, 0.68395, 3.05328, 0.04905],
                        [5.15639, 0.684, 2.66472, 0.0562],
                        [5.30615, 0.68405, 2.39019, 0.06266],
                        [5.45583, 0.68451, 2.17748, 0.06874],
                        [5.60502, 0.68718, 2.01807, 0.07394],
                        [5.7532, 0.69379, 1.88878, 0.07853],
                        [5.89972, 0.70599, 1.76616, 0.08325],
                        [6.04377, 0.72526, 1.67033, 0.08701],
                        [6.18439, 0.75295, 1.58506, 0.09042],
                        [6.32046, 0.79019, 1.52269, 0.09265],
                        [6.45071, 0.83782, 1.47749, 0.09387],
                        [6.57377, 0.89638, 1.44101, 0.09457],
                        [6.68794, 0.96627, 1.41228, 0.09479],
                        [6.79149, 1.04745, 1.4, 0.09398],
                        [6.8825, 1.13948, 1.4, 0.09245],
                        [6.95918, 1.24137, 1.4, 0.09108],
                        [7.01998, 1.35156, 1.4, 0.0899],
                        [7.06356, 1.46817, 1.4, 0.08892],
                        [7.08889, 1.58892, 1.4, 0.08813],
                        [7.09554, 1.71135, 1.40987, 0.08696],
                        [7.08391, 1.83301, 1.42428, 0.08581],
                        [7.05464, 1.95172, 1.44551, 0.08458],
                        [7.00862, 2.06554, 1.48497, 0.08268],
                        [6.94713, 2.17293, 1.5281, 0.08098],
                        [6.87145, 2.2726, 1.59295, 0.07857],
                        [6.78307, 2.36372, 1.66705, 0.07614],
                        [6.68347, 2.44574, 1.75542, 0.0735],
                        [6.57405, 2.5184, 1.85516, 0.0708],
                        [6.45616, 2.5817, 1.97618, 0.06771],
                        [6.33106, 2.63588, 2.0988, 0.06495],
                        [6.19985, 2.68125, 2.25948, 0.06145],
                        [6.06362, 2.71841, 2.43157, 0.05807],
                        [5.9233, 2.74801, 2.64776, 0.05416],
                        [5.77978, 2.77086, 2.88956, 0.05029],
                        [5.63989, 2.79918, 2.94348, 0.04849],
                        [5.50224, 2.83281, 2.9928, 0.04735],
                        [5.36677, 2.87145, 3.02254, 0.04661],
                        [5.23343, 2.91494, 3.06786, 0.04571],
                        [5.10219, 2.96305, 3.10374, 0.04504],
                        [4.973, 3.01563, 3.15563, 0.0442],
                        [4.8458, 3.07251, 3.20953, 0.04341],
                        [4.72056, 3.13353, 3.30108, 0.0422],
                        [4.59719, 3.19846, 3.41546, 0.04082],
                        [4.47558, 3.26703, 3.56505, 0.03916],
                        [4.35562, 3.33896, 3.71196, 0.03768],
                        [4.23719, 3.41401, 3.90123, 0.03594],
                        [4.12017, 3.49188, 3.73873, 0.0376],
                        [4.00444, 3.57232, 3.33353, 0.04228],
                        [3.88988, 3.65504, 3.03918, 0.0465],
                        [3.77636, 3.73981, 2.80534, 0.0505],
                        [3.66497, 3.82546, 2.61653, 0.0537],
                        [3.55239, 3.90877, 2.46065, 0.05692],
                        [3.43824, 3.98902, 2.32422, 0.06003],
                        [3.32216, 4.06547, 2.21026, 0.06288],
                        [3.20379, 4.13738, 2.10618, 0.06576],
                        [3.08278, 4.204, 2.02126, 0.06834],
                        [2.95878, 4.26455, 1.95571, 0.07056],
                        [2.83148, 4.31821, 1.91051, 0.07231],
                        [2.70065, 4.36401, 1.87115, 0.07408],
                        [2.56625, 4.40087, 1.8496, 0.07535],
                        [2.4286, 4.4275, 1.82685, 0.07674],
                        [2.28873, 4.44262, 1.79987, 0.07817],
                        [2.14839, 4.44522, 1.77058, 0.07927],
                        [2.00982, 4.43488, 1.74182, 0.07978],
                        [1.87516, 4.41176, 1.69967, 0.08038],
                        [1.74606, 4.37659, 1.65526, 0.08084],
                        [1.62365, 4.33019, 1.61119, 0.08125],
                        [1.50871, 4.27333, 1.61119, 0.07959],
                        [1.40186, 4.20666, 1.61119, 0.07817],
                        [1.30361, 4.13072, 1.61119, 0.07707],
                        [1.21462, 4.04583, 1.61119, 0.07633],
                        [1.13571, 3.95217, 1.61119, 0.07601],
                        [1.0679, 3.84989, 1.65623, 0.07409],
                        [1.01118, 3.73996, 1.80711, 0.06845],
                        [0.9643, 3.6239, 1.88504, 0.0664],
                        [0.92703, 3.50238, 1.96946, 0.06454],
                        [0.89912, 3.376, 2.06282, 0.06274],
                        [0.88033, 3.2453, 2.16685, 0.06094],
                        [0.87039, 3.1108, 2.27846, 0.05919],
                        [0.86902, 2.97299, 2.4083, 0.05723],
                        [0.87582, 2.83237, 2.55291, 0.05515],
                        [0.89035, 2.68943, 2.68181, 0.05358],
                        [0.91211, 2.54465, 2.52173, 0.05806],
                        [0.94052, 2.39855, 2.38528, 0.0624],
                        [0.97439, 2.254, 2.265, 0.06554],
                        [1.01306, 2.11309, 2.15638, 0.06776],
                        [1.05694, 1.97634, 2.05991, 0.06972],
                        [1.10644, 1.84427, 1.95979, 0.07197],
                        [1.16188, 1.71738, 1.86628, 0.0742],
                        [1.22355, 1.59613, 1.7822, 0.07633],
                        [1.29168, 1.48099, 1.7822, 0.07507],
                        [1.36652, 1.37247, 1.7822, 0.07397],
                        [1.44828, 1.27108, 1.7822, 0.07308],
                        [1.53729, 1.17755, 1.7822, 0.07245],
                        [1.63392, 1.09275, 1.7822, 0.07214],
                        [1.73855, 1.01773, 1.88407, 0.06833],
                        [1.85012, 0.95182, 1.97979, 0.06545],
                        [1.96786, 0.89453, 2.09005, 0.06265],
                        [2.09108, 0.84536, 2.20601, 0.06014],
                        [2.21921, 0.8039, 2.33586, 0.05765],
                        [2.35172, 0.7697, 2.48669, 0.05503],
                        [2.4881, 0.74228, 2.66541, 0.05219],
                        [2.62784, 0.72109, 2.8836, 0.04901],
                        [2.77043, 0.7055, 3.16367, 0.04534],
                        [2.91538, 0.69477, 3.53647, 0.0411]]

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

        ## Constant ##
        DISTANCE_MULTIPLE = 50  # Increased multiple for more strict line following
        SPEED_MULTIPLE = 25  # Increased speed multiple to encourage faster driving
        STEP_MULTIPLE = 25  # Increased step multiple to encourage faster driving
        PROGRESS_MULTIPLE = 100 # Increased progress to encourage completion of laps
        SPEED_DIFF_NO_REWARD = 1
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 10
        FASTEST_TIME = 7
        REWARD_FOR_FASTEST_TIME = 500  # Increase to make completing the lap faster more desirable

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)) ** 2)  # Quadratic punishment for deviation
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
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
        reward += steps_reward * STEP_MULTIPLE

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 10:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        # Calculate reward based on progress
        progress_reward = (progress / 100) * PROGRESS_MULTIPLE
        reward += progress_reward * PROGRESS_MULTIPLE

        # Calculate finish reward based on fastest time
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

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