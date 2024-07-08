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

        racing_track = [[2.08696, 0.99616, 1.42805, 0.06824],
                        [2.18493, 0.9982, 1.48009, 0.0662],
                        [2.28309, 1.00873, 1.54339, 0.06397],
                        [2.38098, 1.0272, 1.62971, 0.06113],
                        [2.47826, 1.05294, 1.73131, 0.05812],
                        [2.57469, 1.08525, 1.84051, 0.05525],
                        [2.67009, 1.12347, 1.96882, 0.0522],
                        [2.76437, 1.16695, 1.95385, 0.05314],
                        [2.85749, 1.21503, 1.90367, 0.05505],
                        [2.94947, 1.26709, 1.86591, 0.05664],
                        [3.04317, 1.31619, 1.84436, 0.05735],
                        [3.13792, 1.36154, 1.83806, 0.05715],
                        [3.23372, 1.40273, 1.83048, 0.05697],
                        [3.33054, 1.43937, 1.83048, 0.05655],
                        [3.42832, 1.47109, 1.83048, 0.05616],
                        [3.527, 1.49759, 1.83048, 0.05582],
                        [3.62645, 1.51863, 1.83048, 0.05553],
                        [3.72654, 1.53403, 1.83048, 0.05533],
                        [3.82713, 1.54369, 1.83048, 0.0552],
                        [3.92805, 1.54758, 1.83048, 0.05517],
                        [4.02914, 1.54564, 1.83874, 0.05499],
                        [4.13023, 1.53789, 1.85651, 0.05461],
                        [4.23113, 1.52439, 1.88387, 0.05404],
                        [4.33168, 1.50526, 1.92085, 0.05328],
                        [4.4317, 1.48065, 1.96905, 0.05231],
                        [4.53104, 1.45076, 1.95077, 0.05318],
                        [4.62955, 1.41586, 1.82161, 0.05737],
                        [4.72709, 1.37618, 1.70115, 0.0619],
                        [4.82353, 1.33194, 1.6051, 0.0661],
                        [4.91872, 1.28336, 1.52298, 0.07017],
                        [5.0125, 1.23067, 1.44542, 0.07442],
                        [5.10076, 1.17655, 1.39571, 0.07418],
                        [5.18921, 1.12752, 1.35374, 0.07471],
                        [5.2775, 1.08422, 1.3254, 0.07419],
                        [5.36537, 1.04706, 1.30979, 0.07283],
                        [5.45262, 1.01639, 1.3, 0.07114],
                        [5.53917, 0.99242, 1.3, 0.06908],
                        [5.62499, 0.97529, 1.3, 0.06732],
                        [5.71007, 0.96523, 1.3, 0.0659],
                        [5.79443, 0.96235, 1.3, 0.06493],
                        [5.87805, 0.96684, 1.3, 0.06442],
                        [5.96087, 0.97893, 1.3, 0.06438],
                        [6.04269, 0.99884, 1.3, 0.06478],
                        [6.12322, 1.02684, 1.308, 0.06518],
                        [6.20201, 1.06307, 1.31407, 0.06599],
                        [6.27847, 1.10767, 1.33264, 0.06642],
                        [6.35188, 1.16058, 1.35463, 0.06681],
                        [6.42146, 1.22161, 1.38405, 0.06687],
                        [6.48642, 1.29034, 1.41933, 0.06662],
                        [6.54605, 1.36615, 1.45903, 0.06611],
                        [6.59977, 1.44835, 1.5066, 0.06518],
                        [6.64722, 1.53615, 1.56584, 0.06373],
                        [6.68822, 1.62873, 1.64084, 0.06171],
                        [6.72286, 1.72529, 1.73332, 0.05918],
                        [6.75142, 1.82507, 1.84577, 0.05623],
                        [6.77431, 1.92743, 1.97862, 0.05301],
                        [6.79207, 2.0318, 2.13229, 0.04965],
                        [6.80525, 2.1377, 2.32681, 0.04587],
                        [6.81448, 2.24476, 2.49088, 0.04314],
                        [6.8204, 2.35262, 2.2381, 0.04827],
                        [6.8237, 2.46102, 2.04345, 0.05307],
                        [6.82507, 2.56974, 1.88854, 0.05757],
                        [6.82519, 2.67859, 1.76159, 0.06179],
                        [6.82427, 2.78736, 1.65535, 0.06571],
                        [6.82146, 2.8959, 1.56538, 0.06936],
                        [6.81591, 3.00399, 1.49245, 0.07252],
                        [6.80681, 3.11135, 1.43725, 0.07496],
                        [6.79338, 3.21764, 1.39478, 0.07681],
                        [6.77489, 3.32247, 1.36204, 0.07816],
                        [6.75063, 3.42538, 1.33755, 0.07905],
                        [6.71994, 3.5258, 1.32106, 0.07949],
                        [6.68225, 3.62307, 1.31317, 0.07944],
                        [6.63707, 3.71638, 1.31317, 0.07895],
                        [6.58409, 3.80482, 1.31317, 0.07851],
                        [6.52325, 3.88737, 1.31317, 0.0781],
                        [6.45477, 3.96301, 1.31317, 0.0777],
                        [6.37912, 4.03072, 1.31317, 0.07731],
                        [6.29699, 4.08954, 1.31317, 0.07693],
                        [6.2093, 4.13871, 1.31317, 0.07656],
                        [6.11705, 4.17767, 1.31468, 0.07617],
                        [6.02132, 4.2061, 1.3264, 0.07529],
                        [5.92319, 4.22398, 1.35004, 0.07388],
                        [5.82365, 4.23149, 1.38225, 0.07222],
                        [5.72358, 4.229, 1.41857, 0.07056],
                        [5.62374, 4.21692, 1.46397, 0.06869],
                        [5.52475, 4.19578, 1.52147, 0.06653],
                        [5.42712, 4.16616, 1.59301, 0.06405],
                        [5.33121, 4.12876, 1.67277, 0.06154],
                        [5.23728, 4.08424, 1.75802, 0.05913],
                        [5.14551, 4.03323, 1.76525, 0.05948],
                        [5.056, 3.97634, 1.74101, 0.06092],
                        [4.96882, 3.91411, 1.72355, 0.06215],
                        [4.87828, 3.85591, 1.72107, 0.06254],
                        [4.78538, 3.80279, 1.72107, 0.06218],
                        [4.69026, 3.75517, 1.72107, 0.06181],
                        [4.5931, 3.71345, 1.72107, 0.06143],
                        [4.49415, 3.67798, 1.72107, 0.06107],
                        [4.39369, 3.64911, 1.72107, 0.06074],
                        [4.29203, 3.62702, 1.72107, 0.06044],
                        [4.18952, 3.61184, 1.72107, 0.06021],
                        [4.08651, 3.60357, 1.72537, 0.0599],
                        [3.98331, 3.60215, 1.74919, 0.059],
                        [3.88026, 3.60739, 1.77309, 0.0582],
                        [3.7776, 3.61911, 1.81491, 0.05693],
                        [3.6756, 3.63702, 1.86788, 0.05544],
                        [3.57444, 3.66078, 1.93177, 0.05379],
                        [3.47427, 3.69003, 2.00315, 0.05209],
                        [3.37522, 3.72441, 2.06751, 0.05071],
                        [3.27735, 3.76348, 1.87191, 0.05629],
                        [3.1807, 3.80678, 1.72516, 0.06139],
                        [3.08528, 3.85396, 1.60764, 0.06621],
                        [2.99107, 3.90462, 1.5146, 0.07062],
                        [2.89808, 3.9584, 1.44848, 0.07417],
                        [2.80485, 4.00886, 1.39917, 0.07577],
                        [2.71058, 4.05542, 1.36355, 0.0771],
                        [2.6152, 4.09709, 1.34086, 0.07763],
                        [2.51873, 4.1329, 1.32652, 0.07758],
                        [2.42129, 4.16193, 1.32186, 0.07691],
                        [2.32316, 4.18333, 1.32186, 0.07598],
                        [2.22473, 4.19634, 1.32186, 0.07511],
                        [2.12654, 4.20039, 1.32186, 0.07435],
                        [2.02923, 4.19508, 1.32186, 0.07373],
                        [1.93352, 4.18017, 1.32186, 0.07328],
                        [1.84019, 4.15559, 1.32186, 0.07301],
                        [1.75006, 4.12141, 1.32186, 0.07293],
                        [1.66392, 4.07786, 1.32662, 0.07276],
                        [1.58256, 4.02531, 1.34074, 0.07224],
                        [1.50664, 3.96432, 1.36382, 0.0714],
                        [1.43675, 3.89553, 1.39534, 0.07028],
                        [1.37331, 3.81969, 1.43559, 0.06887],
                        [1.31662, 3.73757, 1.48385, 0.06725],
                        [1.26681, 3.64997, 1.53832, 0.06551],
                        [1.2239, 3.55766, 1.60768, 0.06332],
                        [1.18774, 3.46137, 1.69401, 0.06072],
                        [1.158, 3.36183, 1.80564, 0.05754],
                        [1.13418, 3.25968, 1.93554, 0.05419],
                        [1.11574, 3.1555, 2.08564, 0.05073],
                        [1.10207, 3.04975, 2.27478, 0.04688],
                        [1.09254, 2.94283, 2.42409, 0.04428],
                        [1.08646, 2.83507, 2.16752, 0.04979],
                        [1.08313, 2.72676, 1.97566, 0.05485],
                        [1.08185, 2.61809, 1.82487, 0.05955],
                        [1.08192, 2.5092, 1.70284, 0.06394],
                        [1.08299, 2.40034, 1.60229, 0.06795],
                        [1.08602, 2.29174, 1.5197, 0.07149],
                        [1.09191, 2.18366, 1.45678, 0.0743],
                        [1.10155, 2.07642, 1.40779, 0.07648],
                        [1.11577, 1.97043, 1.37008, 0.07805],
                        [1.13535, 1.86617, 1.34224, 0.07904],
                        [1.16099, 1.76418, 1.32371, 0.07944],
                        [1.19331, 1.66513, 1.31525, 0.07921],
                        [1.23281, 1.56978, 1.31525, 0.07847],
                        [1.27983, 1.479, 1.31525, 0.07774],
                        [1.33448, 1.39369, 1.31525, 0.07703],
                        [1.39662, 1.3148, 1.31525, 0.07635],
                        [1.46589, 1.24325, 1.31525, 0.07571],
                        [1.5417, 1.17989, 1.31525, 0.07512],
                        [1.62332, 1.12541, 1.31525, 0.07461],
                        [1.70986, 1.08033, 1.31813, 0.07403],
                        [1.80039, 1.04492, 1.33206, 0.07298],
                        [1.89397, 1.01921, 1.35613, 0.07157],
                        [1.98976, 1.00306, 1.39228, 0.06977]]

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
