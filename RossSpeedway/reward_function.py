import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            closest_index = distances.index(min(distances))
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            try:
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

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

            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            track_direction = math.degrees(track_direction)

            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        def indexes_cyclical(start, end, array_len):
            start = int(start or 0)
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

        racing_track = [[5.04771, 0.73385, 3.13949, 0.05467],
                        [5.04771, 0.86385, 2.84914, 0.04563],
                        [5.0477, 0.99385, 2.62322, 0.04956],
                        [5.04768, 1.1655, 2.43508, 0.07049],
                        [5.04577, 1.46647, 2.27053, 0.13256],
                        [5.0385, 1.76546, 2.1315, 0.14031],
                        [5.0231, 2.06125, 2.00742, 0.14754],
                        [4.99699, 2.35251, 1.86932, 0.15644],
                        [4.95784, 2.63788, 1.86932, 0.15409],
                        [4.90368, 2.91594, 1.86932, 0.15155],
                        [4.8328, 3.18523, 1.86932, 0.14896],
                        [4.74366, 3.44416, 1.86932, 0.14649],
                        [4.63479, 3.6909, 1.86932, 0.14427],
                        [4.50489, 3.92341, 1.86932, 0.14248],
                        [4.35268, 4.13919, 1.86932, 0.14126],
                        [4.17636, 4.33443, 1.95464, 0.13459],
                        [3.97929, 4.50954, 2.04624, 0.12883],
                        [3.76421, 4.66498, 2.14729, 0.12359],
                        [3.53342, 4.80134, 2.25343, 0.11896],
                        [3.28888, 4.91918, 2.37449, 0.11432],
                        [3.03237, 5.01921, 2.51029, 0.10967],
                        [2.76556, 5.10224, 2.66913, 0.10469],
                        [2.49001, 5.16931, 2.85859, 0.09921],
                        [2.20719, 5.22169, 3.08947, 0.0931],
                        [1.91852, 5.26084, 3.37319, 0.08636],
                        [1.62527, 5.28843, 3.74989, 0.07855],
                        [1.32861, 5.30634, 3.54826, 0.08376],
                        [1.02958, 5.31661, 3.16128, 0.09465],
                        [0.72904, 5.32133, 2.86896, 0.10477],
                        [0.42768, 5.32264, 2.65785, 0.11339],
                        [0.12603, 5.32269, 2.47914, 0.12167],
                        [-0.17527, 5.32005, 2.32953, 0.12934],
                        [-0.47578, 5.31186, 2.2076, 0.13618],
                        [-0.7749, 5.29525, 2.11365, 0.14174],
                        [-1.07183, 5.26747, 2.03779, 0.14635],
                        [-1.36545, 5.22589, 1.96699, 0.15076],
                        [-1.65417, 5.16801, 1.89545, 0.15535],
                        [-1.936, 5.09184, 1.83003, 0.15953],
                        [-2.2084, 4.99573, 1.76731, 0.16345],
                        [-2.46837, 4.87855, 1.7068, 0.16707],
                        [-2.71266, 4.73998, 1.64191, 0.17105],
                        [-2.93811, 4.58064, 1.58079, 0.17464],
                        [-3.14183, 4.40189, 1.51689, 0.17867],
                        [-3.32112, 4.20555, 1.4557, 0.18265],
                        [-3.47333, 3.9938, 1.39666, 0.18671],
                        [-3.5961, 3.76925, 1.39666, 0.18324],
                        [-3.68727, 3.53481, 1.39666, 0.1801],
                        [-3.74481, 3.29375, 1.39666, 0.17744],
                        [-3.76649, 3.04971, 1.39666, 0.17542],
                        [-3.75016, 2.80693, 1.39666, 0.17422],
                        [-3.6934, 2.57054, 1.39666, 0.17406],
                        [-3.59382, 2.34708, 1.39666, 0.17516],
                        [-3.44963, 2.14536, 1.61227, 0.15379],
                        [-3.27364, 1.96525, 1.7173, 0.14664],
                        [-3.07064, 1.80786, 1.84572, 0.13917],
                        [-2.84503, 1.67323, 1.99388, 0.13177],
                        [-2.60062, 1.56071, 2.1828, 0.12327],
                        [-2.341, 1.46874, 2.42196, 0.11372],
                        [-2.06948, 1.39493, 2.15698, 0.13045],
                        [-1.78915, 1.33598, 1.96137, 0.14605],
                        [-1.50274, 1.28813, 1.8093, 0.16049],
                        [-1.21285, 1.24703, 1.68767, 0.17349],
                        [-0.92838, 1.2012, 1.57507, 0.18294],
                        [-0.64996, 1.1454, 1.48012, 0.19185],
                        [-0.38086, 1.07538, 1.39687, 0.19906],
                        [-0.12445, 0.98782, 1.32154, 0.20502],
                        [0.11583, 0.88036, 1.25, 0.21058],
                        [0.33648, 0.7517, 1.25, 0.20434],
                        [0.53388, 0.60149, 1.25, 0.19844],
                        [0.7043, 0.43039, 1.25, 0.19319],
                        [0.84331, 0.23973, 1.25, 0.18877],
                        [0.94616, 0.03204, 1.25, 0.18541],
                        [1.00759, -0.18864, 1.25, 0.18326],
                        [1.02188, -0.41622, 1.25, 0.18242],
                        [0.98318, -0.64123, 1.40423, 0.16259],
                        [0.90316, -0.85668, 1.47332, 0.15599],
                        [0.78625, -1.05825, 1.53327, 0.15198],
                        [0.63574, -1.24239, 1.62002, 0.1468],
                        [0.45551, -1.40673, 1.72045, 0.14177],
                        [0.24921, -1.54971, 1.8319, 0.13702],
                        [0.02026, -1.67038, 1.9641, 0.13177],
                        [-0.22801, -1.76853, 2.12728, 0.12549],
                        [-0.49223, -1.84481, 2.33105, 0.11798],
                        [-0.76918, -1.90077, 2.57665, 0.10965],
                        [-1.05572, -1.9389, 2.32028, 0.12458],
                        [-1.34898, -1.96267, 2.11918, 0.13884],
                        [-1.64653, -1.97604, 1.96218, 0.1518],
                        [-1.94628, -1.98353, 1.82647, 0.16417],
                        [-2.24328, -1.99776, 1.71224, 0.17366],
                        [-2.5353, -2.02302, 1.61063, 0.18198],
                        [-2.81979, -2.06305, 1.5968, 0.17992],
                        [-3.09404, -2.12089, 1.54116, 0.18186],
                        [-3.35518, -2.19871, 1.47504, 0.18474],
                        [-3.60021, -2.29796, 1.41433, 0.18692],
                        [-3.82606, -2.41918, 1.41433, 0.18123],
                        [-4.02944, -2.56229, 1.41433, 0.17583],
                        [-4.20689, -2.72645, 1.41433, 0.17092],
                        [-4.35462, -2.91001, 1.41433, 0.1666],
                        [-4.4718, -3.10917, 1.41433, 0.16338],
                        [-4.55585, -3.32095, 1.41433, 0.16111],
                        [-4.6033, -3.54194, 1.41433, 0.15981],
                        [-4.61041, -3.76744, 1.43066, 0.1577],
                        [-4.57774, -3.99173, 1.45061, 0.15625],
                        [-4.50813, -4.21004, 1.45061, 0.15796],
                        [-4.40079, -4.41726, 1.45061, 0.16088],
                        [-4.25454, -4.60685, 1.46428, 0.16353],
                        [-4.07104, -4.77202, 1.72127, 0.14343],
                        [-3.86155, -4.91482, 1.83233, 0.13836],
                        [-3.62972, -5.03441, 1.95914, 0.13315],
                        [-3.37893, -5.13062, 2.11624, 0.12693],
                        [-3.11252, -5.20419, 2.31633, 0.11932],
                        [-2.83372, -5.25673, 2.57754, 0.11007],
                        [-2.54553, -5.29075, 2.95167, 0.09831],
                        [-2.25075, -5.30963, 3.53882, 0.08347],
                        [-1.95182, -5.31745, 4.0, 0.07476],
                        [-1.65075, -5.31873, 4.0, 0.07527],
                        [-1.34911, -5.31818, 4.0, 0.07541],
                        [-1.04746, -5.31762, 4.0, 0.07541],
                        [-0.74582, -5.31704, 4.0, 0.07541],
                        [-0.44418, -5.31646, 4.0, 0.07541],
                        [-0.14253, -5.31589, 3.52825, 0.08549],
                        [0.15911, -5.31532, 3.12698, 0.09647],
                        [0.46076, -5.31475, 2.83698, 0.10633],
                        [0.7624, -5.31418, 2.60458, 0.11581],
                        [1.06405, -5.31359, 2.41396, 0.12496],
                        [1.3649, -5.3108, 2.25289, 0.13355],
                        [1.66379, -5.30278, 2.11455, 0.1414],
                        [1.95942, -5.28664, 1.99291, 0.14856],
                        [2.25035, -5.25976, 1.87987, 0.15542],
                        [2.53513, -5.21981, 1.77855, 0.16168],
                        [2.81224, -5.16486, 1.77855, 0.15884],
                        [3.08009, -5.09318, 1.77855, 0.1559],
                        [3.33702, -5.00329, 1.77855, 0.15304],
                        [3.58118, -4.89388, 1.77855, 0.15043],
                        [3.81055, -4.76379, 1.77855, 0.14826],
                        [4.02275, -4.61194, 1.77855, 0.14672],
                        [4.21477, -4.43714, 1.77855, 0.14599],
                        [4.38268, -4.23834, 1.94033, 0.13411],
                        [4.52879, -4.02049, 2.0484, 0.12806],
                        [4.65394, -3.78618, 2.16819, 0.12251],
                        [4.75898, -3.53762, 2.30696, 0.11697],
                        [4.84495, -3.27677, 2.46481, 0.11143],
                        [4.913, -3.00541, 2.65593, 0.10533],
                        [4.96454, -2.72526, 2.89795, 0.09829],
                        [5.00136, -2.43798, 3.21293, 0.09014],
                        [5.02556, -2.14517, 3.65178, 0.08046],
                        [5.03957, -1.84831, 4.0, 0.0743],
                        [5.04607, -1.54876, 4.0, 0.07491],
                        [5.04785, -1.24766, 4.0, 0.07527],
                        [5.04784, -0.94602, 4.0, 0.07541],
                        [5.04781, -0.64437, 4.0, 0.07541],
                        [5.04779, -0.34273, 4.0, 0.07541],
                        [5.04777, -0.04108, 4.0, 0.07541],
                        [5.04775, 0.26056, 4.0, 0.07541],
                        [5.04772, 0.56221, 3.54021, 0.08521]]

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

        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
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

        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

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

        return float(reward)


reward_object = Reward()


def reward_function(params):
    return reward_object.reward_function(params)
